/*
 * V4L2 Memory-to-Memory hardware decoder backend for Moonlight Embedded.
 *
 * This file is part of Moonlight Embedded.
 *
 * Moonlight is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Two consumption models are supported:
 *   1. Zero copy: dequeue_raw() -> export the DMA-BUF -> scan it out on a
 *      display plane -> release() once the flip has completed.
 *   2. CPU copy: dequeue() converts NV12 to planar YUV for SDL. This is
 *      only a fallback: the driver maps capture buffers uncached, so reading
 *      them from the CPU costs roughly 25ms per 1080p frame on an A53.
 */

#include "v4l2m2m.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define HAVE_NEON 1
#endif

#define DEVICE_PREFIX "/dev/video"
#define DEVICE_MAX    64
#define OUT_BUFFERS  16
#define CAP_BUFFERS  20
#define MAX_PLANES   2
#define NUM_FRAMES   2
/* How long v4l2m2m_feed() may block waiting for the decoder to hand an
 * input buffer back. FFmpeg waits indefinitely; 200ms is enough for HEVC
 * startup on meson-vdec without hanging real streams. */
#define BACKPRESSURE_MS 200

typedef struct {
  void* start[MAX_PLANES];
  size_t length[MAX_PLANES];
  int dmabuf[MAX_PLANES];
} buffer_t;

static int fd = -1;
/* Node we ended up on, for logging and for the "is this a decoder" probe. */
static char dev_path[32] = DEVICE_PREFIX "0";
/* Number of planes in the negotiated capture format: 2 for NV12M, 1 for the
 * single-planar NV12 layout some decoders expose instead. */
static int cap_planes = MAX_PLANES;
static buffer_t out_bufs[OUT_BUFFERS];
static buffer_t cap_bufs[CAP_BUFFERS];
static int out_free[OUT_BUFFERS];
static int out_count, cap_count;
static int cap_w, cap_h;
static int cap_stride[MAX_PLANES];
static int last_error_flag;
static int cap_started;
/* coded size of the compressed stream, as passed to init_codec(); used for
 * the S_SELECTION call that mirrors FFmpeg's v4l2_try_start(). */
static int coded_w, coded_h;
/* last size reported by G_FMT on the capture queue; source-change events
 * are compared against this rather than against the (possibly cropped)
 * visible size so a plain re-negotiation is not mistaken for a resize. */
static int gfmt_w, gfmt_h;

static unsigned char* frames[NUM_FRAMES];
static unsigned char* frame_planes[NUM_FRAMES][3];
static int frame_strides[NUM_FRAMES][3];
static size_t frames_size;
static int frame_index;   /* most recently produced */
static int frame_next;    /* buffer to write next */

static int xioctl(int f, unsigned long req, void* arg) {
  int r;
  do {
    r = ioctl(f, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

static int queue_capture(int index) {
  struct v4l2_buffer buf;
  struct v4l2_plane planes[MAX_PLANES];
  memset(&buf, 0, sizeof(buf));
  memset(planes, 0, sizeof(planes));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = index;
  buf.length = MAX_PLANES;
  buf.m.planes = planes;
  return xioctl(fd, VIDIOC_QBUF, &buf);
}

static int request_and_map(buffer_t* bufs, int* count, enum v4l2_buf_type type,
                           int num_planes) {
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = *count;
  req.type = type;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    perror("VIDIOC_REQBUFS");
    return -1;
  }
  if (req.count < 2) {
    fprintf(stderr, "V4L2: insufficient buffers on %s\n",
            type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ? "output" : "capture");
    return -1;
  }
  *count = req.count;

  for (int i = 0; i < *count; i++) {
    struct v4l2_buffer buf;
    struct v4l2_plane planes[MAX_PLANES];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type = type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    buf.length = num_planes;
    buf.m.planes = planes;
    if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
      perror("VIDIOC_QUERYBUF");
      return -1;
    }
    for (int j = 0; j < num_planes; j++) {
      bufs[i].length[j] = planes[j].length;
      bufs[i].dmabuf[j] = -1;
      bufs[i].start[j] = mmap(NULL, planes[j].length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, planes[j].m.mem_offset);
      if (bufs[i].start[j] == MAP_FAILED) {
        perror("mmap");
        bufs[i].start[j] = NULL;
        return -1;
      }
    }
  }
  return 0;
}

/*
 * FFmpeg's v4l2_m2m_dec starts the capture side only after the first
 * compressed packet has been queued: G_FMT (the driver now knows the real
 * coded size), S_SELECTION with the coded size, REQBUFS + QBUF + STREAMON.
 * meson-vdec's HEVC state machine rejects the capture queue being started
 * before that point.
 */
static int start_capture(void) {
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
    perror("G_FMT capture");
    return -1;
  }
  cap_w = fmt.fmt.pix_mp.width;
  cap_h = fmt.fmt.pix_mp.height;
  gfmt_w = cap_w;
  gfmt_h = cap_h;
  cap_stride[0] = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
  cap_stride[1] = fmt.fmt.pix_mp.plane_fmt[1].bytesperline;

  /* FFmpeg sets the crop to the coded size and reads the visible rectangle
   * back with G_SELECTION; some drivers only report the real coded size
   * there. Failure is harmless (single-planar selection target is not
   * universally supported), exactly as in v4l2_try_start(). */
  if (coded_w > 0 && coded_h > 0) {
    struct v4l2_selection sel;
    memset(&sel, 0, sizeof(sel));
    sel.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sel.r.width = coded_w;
    sel.r.height = coded_h;
    if (xioctl(fd, VIDIOC_S_SELECTION, &sel) == 0) {
      struct v4l2_selection gs;
      memset(&gs, 0, sizeof(gs));
      gs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd, VIDIOC_G_SELECTION, &gs) == 0 &&
          gs.r.width > 0 && gs.r.height > 0) {
        cap_w = gs.r.width;
        cap_h = gs.r.height;
      }
    }
  }

  cap_count = CAP_BUFFERS;
  if (request_and_map(cap_bufs, &cap_count, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                      cap_planes) < 0)
    return -1;
  for (int i = 0; i < cap_count; i++) {
    if (queue_capture(i) < 0) {
      perror("QBUF capture");
      return -1;
    }
  }
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    perror("STREAMON capture");
    return -1;
  }
  cap_started = 1;
  fprintf(stderr, "V4L2: capture started %dx%d stride %d/%d buffers=%d\n",
          cap_w, cap_h, cap_stride[0], cap_stride[1], cap_count);
  return 0;
}

/* Undo a partial or failed start_capture() so it can be retried on the next
 * packet, the way FFmpeg's v4l2_try_start() is retried per frame. */
static void reset_capture(void) {
  if (fd >= 0 && cap_count > 0) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = 0;
    xioctl(fd, VIDIOC_REQBUFS, &req);
  }
  for (int i = 0; i < cap_count && i < CAP_BUFFERS; i++) {
    for (int j = 0; j < cap_planes; j++) {
      if (cap_bufs[i].dmabuf[j] >= 0) {
        close(cap_bufs[i].dmabuf[j]);
        cap_bufs[i].dmabuf[j] = -1;
      }
      if (cap_bufs[i].start[j] != NULL && cap_bufs[i].start[j] != MAP_FAILED) {
        munmap(cap_bufs[i].start[j], cap_bufs[i].length[j]);
        cap_bufs[i].start[j] = NULL;
      }
    }
  }
  cap_count = 0;
  cap_started = 0;
}

/* Does this node enumerate `pixelformat` on its output (coded) queue? */
static int output_has_format(int f, unsigned int pixelformat) {
  struct v4l2_fmtdesc desc;
  for (unsigned int i = 0;; i++) {
    memset(&desc, 0, sizeof(desc));
    desc.index = i;
    desc.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (xioctl(f, VIDIOC_ENUM_FMT, &desc) < 0)
      return 0;
    if (desc.pixelformat == pixelformat)
      return 1;
  }
}

/*
 * Find a usable V4L2 memory-to-memory decoder. Any /dev/video* that offers a
 * multiplanar M2M streaming interface and decodes `codec` qualifies; the node
 * number is not fixed, meson-vdec is video0 here but other SoCs differ.
 * A negative `codec` accepts any format this backend can feed.
 */
static int open_decoder(int codec) {
  char path[32];
  for (int i = 0; i < DEVICE_MAX; i++) {
    snprintf(path, sizeof(path), DEVICE_PREFIX "%d", i);
    if (access(path, R_OK | W_OK) != 0)
      continue;

    int f = open(path, O_RDWR | O_NONBLOCK);
    if (f < 0)
      continue;

    struct v4l2_capability cap;
    if (xioctl(f, VIDIOC_QUERYCAP, &cap) < 0 ||
        !(cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) ||
        !(cap.capabilities & V4L2_CAP_STREAMING)) {
      close(f);
      continue;
    }

    int ok;
    if (codec == V4L2M2M_CODEC_HEVC)
      ok = output_has_format(f, V4L2_PIX_FMT_HEVC);
    else if (codec == V4L2M2M_CODEC_H264)
      ok = output_has_format(f, V4L2_PIX_FMT_H264);
    else
      ok = output_has_format(f, V4L2_PIX_FMT_H264) ||
           output_has_format(f, V4L2_PIX_FMT_HEVC);
    if (!ok) {
      close(f);
      continue;
    }

    snprintf(dev_path, sizeof(dev_path), "%s", path);
    return f;
  }
  return -1;
}

int v4l2m2m_available(void) {
  const char* env = getenv("MOONLIGHT_FORCE_SW");
  if (env != NULL && env[0] != '\0' && strcmp(env, "0") != 0)
    return 0;

  int f = open_decoder(-1);
  if (f < 0)
    return 0;
  close(f);
  return 1;
}

/*
 * Negotiate the capture format. meson-vdec only offers two-plane NV12M, while
 * V4L2 M2M decoders on other SoCs may expose single-planar NV12, so try the
 * layout that can be scanned out directly first and fall back once.
 */
static int setup_capture(int width, int height) {
  static const struct {
    unsigned int format;
    int planes;
  } candidates[] = {
    { V4L2_PIX_FMT_NV12M, 2 },
    { V4L2_PIX_FMT_NV12, 1 },
  };

  for (unsigned int i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = candidates[i].format;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    fmt.fmt.pix_mp.num_planes = candidates[i].planes;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
      continue;

    struct v4l2_format got;
    memset(&got, 0, sizeof(got));
    got.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(fd, VIDIOC_G_FMT, &got) < 0)
      continue;
    /* The driver may substitute another format; only accept an exact match. */
    if (got.fmt.pix_mp.pixelformat != candidates[i].format)
      continue;

    cap_planes = got.fmt.pix_mp.num_planes;
    if (cap_planes < 1 || cap_planes > MAX_PLANES)
      continue;

    cap_w = got.fmt.pix_mp.width;
    cap_h = got.fmt.pix_mp.height;
    cap_stride[0] = got.fmt.pix_mp.plane_fmt[0].bytesperline;
    cap_stride[1] = (cap_planes > 1)
                        ? got.fmt.pix_mp.plane_fmt[1].bytesperline
                        : cap_stride[0];
    return 0;
  }

  fprintf(stderr, "V4L2: capture side offers neither NV12M nor NV12\n");
  return -1;
}

int v4l2m2m_init_codec(int width, int height, int codec) {
  if (codec != V4L2M2M_CODEC_H264 && codec != V4L2M2M_CODEC_HEVC) {
    fprintf(stderr, "V4L2: unsupported codec %d\n", codec);
    return -1;
  }

  v4l2m2m_cleanup();

  coded_w = width;
  coded_h = height;

  fd = open_decoder(codec);
  if (fd < 0) {
    fprintf(stderr, "V4L2: no memory-to-memory decoder for %s found\n",
            codec == V4L2M2M_CODEC_HEVC ? "HEVC" : "H.264");
    return -1;
  }
  fprintf(stderr, "V4L2: using %s\n", dev_path);

  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  fmt.fmt.pix_mp.width = width;
  fmt.fmt.pix_mp.height = height;
  fmt.fmt.pix_mp.pixelformat = codec == V4L2M2M_CODEC_HEVC ?
                               V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264;
  fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
  fmt.fmt.pix_mp.num_planes = 1;
  fmt.fmt.pix_mp.plane_fmt[0].sizeimage = width * height;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    perror("VIDIOC_S_FMT output");
    goto fail;
  }

  if (setup_capture(width, height) < 0)
    goto fail;

  out_count = OUT_BUFFERS;
  cap_count = 0;
  if (request_and_map(out_bufs, &out_count, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 1) < 0)
    goto fail;
  fprintf(stderr, "V4L2: %s output buffers=%d (capture starts on first packet)\n",
          codec == V4L2M2M_CODEC_HEVC ? "HEVC" : "H264", out_count);

  for (int i = 0; i < out_count; i++)
    out_free[i] = 1;

  /* Stateful decoders report their parsed stream properties through events;
   * meson-vdec's HEVC path expects this subscription. FFmpeg also subscribes
   * to EOS; it is harmless when unsupported. */
  struct v4l2_event_subscription sub;
  memset(&sub, 0, sizeof(sub));
  sub.type = V4L2_EVENT_SOURCE_CHANGE;
  if (xioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &sub) < 0)
    perror("SUBSCRIBE_EVENT source change (continuing)");
  memset(&sub, 0, sizeof(sub));
  sub.type = V4L2_EVENT_EOS;
  if (xioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &sub) < 0)
    perror("SUBSCRIBE_EVENT eos (continuing)");

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    perror("STREAMON output");
    goto fail;
  }

  return 0;

fail:
  v4l2m2m_cleanup();
  return -1;
}

int v4l2m2m_init(int width, int height) {
  return v4l2m2m_init_codec(width, height, V4L2M2M_CODEC_H264);
}

static int handle_decoder_events(void);

/* Reclaim every output buffer the decoder has already consumed. */
static int reclaim_output(int* found) {
  int got = 0;
  for (;;) {
    struct v4l2_buffer buf;
    struct v4l2_plane planes[MAX_PLANES];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = 1;
    buf.m.planes = planes;
    if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0)
      break;
    if (buf.index >= out_count)
      break;
    out_free[buf.index] = 1;
    got = 1;
  }
  if (got) {
    for (int i = 0; i < out_count; i++) {
      if (out_free[i]) {
        *found = i;
        return 0;
      }
    }
  }
  return 0;
}

int v4l2m2m_feed(const unsigned char* data, int length) {
  if (fd < 0 || length <= 0)
    return -1;

  int index = -1;
  for (int i = 0; i < out_count; i++) {
    if (out_free[i]) {
      index = i;
      break;
    }
  }

  /* FFmpeg blocks indefinitely until the decoder returns an input buffer.
   * HEVC startup on meson-vdec can hold the 16 output buffers for well over
   * 20ms, so poll for up to BACKPRESSURE_MS while servicing events and
   * reclaiming consumed buffers instead of failing immediately. */
  for (int waited = 0; index < 0 && waited < BACKPRESSURE_MS; waited += 20) {
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLOUT | POLLPRI;
    pfd.revents = 0;
    int pr = poll(&pfd, 1, 20);
    if (pr < 0 && errno != EINTR)
      return -1;
    if ((pfd.revents & POLLPRI) && handle_decoder_events() < 0)
      return -1;
    reclaim_output(&index);
  }

  if (index < 0) {
    fprintf(stderr, "V4L2: no free output buffer after %dms "
                    "(out_count=%d cap_started=%d)\n",
            BACKPRESSURE_MS, out_count, cap_started);
    return -1;
  }

  size_t capacity = out_bufs[index].length[0];
  if ((size_t)length > capacity) {
    fprintf(stderr, "V4L2: packet of %d bytes exceeds output buffer %zu\n",
            length, capacity);
    length = (int)capacity;
  }
  memcpy(out_bufs[index].start[0], data, length);

  struct v4l2_buffer buf;
  struct v4l2_plane planes[MAX_PLANES];
  memset(&buf, 0, sizeof(buf));
  memset(planes, 0, sizeof(planes));
  buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = index;
  buf.length = 1;
  buf.m.planes = planes;
  planes[0].bytesused = length;
  if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
    perror("QBUF output");
    out_free[index] = 1;
    return -1;
  }
  out_free[index] = 0;
  if (!cap_started) {
    if (start_capture() < 0) {
      /* FFmpeg retries v4l2_try_start on every packet and only treats
       * ENOMEM as fatal: the driver usually just has not parsed the stream
       * header yet. Undo the partial setup and retry next packet. */
      reset_capture();
    }
  }
  return 0;
}

static int handle_decoder_events(void) {
  for (;;) {
    struct v4l2_event event;
    memset(&event, 0, sizeof(event));
    if (xioctl(fd, VIDIOC_DQEVENT, &event) < 0) {
      if (errno == EAGAIN || errno == ENOENT)
        return 0;
      perror("DQEVENT");
      return -1;
    }

    if (event.type == V4L2_EVENT_SOURCE_CHANGE) {
      struct v4l2_format fmt;
      memset(&fmt, 0, sizeof(fmt));
      fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("G_FMT after decoder event");
        return -1;
      }
      /* Moonlight renegotiates out of band. Keep the existing framebuffer
       * set for the normal same-resolution event; a true mid-stream resize
       * requires a fresh decoder setup rather than silently cropping it. */
      if ((int)fmt.fmt.pix_mp.width != gfmt_w ||
          (int)fmt.fmt.pix_mp.height != gfmt_h) {
        fprintf(stderr, "V4L2: mid-stream resize %ux%u -> %ux%u unsupported\n",
                gfmt_w, gfmt_h, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height);
        return -1;
      }
      gfmt_w = fmt.fmt.pix_mp.width;
      gfmt_h = fmt.fmt.pix_mp.height;
      cap_stride[0] = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
      cap_stride[1] = fmt.fmt.pix_mp.plane_fmt[1].bytesperline;
    }
  }
}

/*
 * Waits for a decoded frame and returns its capture buffer index without
 * touching the pixels and without re-queuing it. The caller must hand the
 * buffer back with v4l2m2m_release() once it is done scanning it out.
 *
 * Returns the index (>= 0), 0 on timeout, -1 on fatal error.
 */
int v4l2m2m_dequeue_raw(int timeout_ms) {
  if (fd < 0)
    return -1;

  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN | POLLPRI;
  pfd.revents = 0;
  int ready = poll(&pfd, 1, timeout_ms);
  if (ready < 0)
    return errno == EINTR ? 0 : -1;
  if (ready == 0)
    return 0;
  if ((pfd.revents & POLLPRI) && handle_decoder_events() < 0)
    return -1;
  if (!(pfd.revents & POLLIN))
    return 0;

  struct v4l2_buffer buf;
  struct v4l2_plane planes[MAX_PLANES];
  memset(&buf, 0, sizeof(buf));
  memset(planes, 0, sizeof(planes));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.length = MAX_PLANES;
  buf.m.planes = planes;
  if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
    if (errno == EAGAIN)
      return 0;
    perror("DQBUF capture");
    return -1;
  }

  last_error_flag = (buf.flags & V4L2_BUF_FLAG_ERROR) ||
                    planes[0].bytesused == 0;
  return buf.index;
}

int v4l2m2m_release(int index) {
  if (fd < 0 || index < 0 || index >= cap_count)
    return -1;
  return queue_capture(index);
}

int v4l2m2m_had_error(void) {
  return last_error_flag;
}

/*
 * Returns a DMA-BUF fd for one plane of a capture buffer, exporting it on
 * first use. Each NV12M plane lives in its own allocation, so the two planes
 * of one buffer are two independent DMA-BUFs.
 */
int v4l2m2m_dmabuf_fd(int index, int plane) {
  if (fd < 0 || index < 0 || index >= cap_count || plane < 0 || plane >= cap_planes)
    return -1;
  if (cap_bufs[index].dmabuf[plane] >= 0)
    return cap_bufs[index].dmabuf[plane];

  struct v4l2_exportbuffer exp;
  memset(&exp, 0, sizeof(exp));
  exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  exp.index = index;
  exp.plane = plane;
  exp.flags = O_RDWR;
  if (xioctl(fd, VIDIOC_EXPBUF, &exp) < 0) {
    perror("VIDIOC_EXPBUF");
    return -1;
  }
  cap_bufs[index].dmabuf[plane] = exp.fd;
  return exp.fd;
}

void* v4l2m2m_plane_ptr(int index, int plane) {
  if (index < 0 || index >= cap_count || plane < 0 || plane >= cap_planes)
    return NULL;
  return cap_bufs[index].start[plane];
}

int v4l2m2m_plane_len(int index, int plane) {
  if (index < 0 || index >= cap_count || plane < 0 || plane >= cap_planes)
    return -1;
  return (int)cap_bufs[index].length[plane];
}

void v4l2m2m_geometry(int* w, int* h, int* stride0, int* stride1) {
  if (w) *w = cap_w;
  if (h) *h = cap_h;
  if (stride0) *stride0 = cap_stride[0] > 0 ? cap_stride[0] : cap_w;
  if (stride1) *stride1 = cap_stride[1] > 0 ? cap_stride[1] : cap_w;
}

int v4l2m2m_buffer_count(void) {
  return cap_count;
}

int v4l2m2m_decoder_fd(void) {
  return fd;
}

/* Planes in the negotiated capture format (2 = NV12M, 1 = single-plane NV12). */
int v4l2m2m_plane_count(void) {
  return cap_planes;
}

const char* v4l2m2m_device_path(void) {
  return dev_path;
}

/* ------------------------------------------------------------------ */
/* CPU conversion fallback: NM12 -> planar YUV420P                     */
/* ------------------------------------------------------------------ */

static int prof_enabled = -1;
static int prof_count;
static double prof_y_ms, prof_uv_ms;

static double prof_now(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static int convert_frame(int index, int stride0, int stride1) {
  int w = cap_w, h = cap_h;
  size_t need = (size_t)w * h * 3 / 2;

  if (frames_size < need) {
    for (int i = 0; i < NUM_FRAMES; i++) {
      free(frames[i]);
      frames[i] = malloc(need);
      if (frames[i] == NULL) {
        frames_size = 0;
        fprintf(stderr, "V4L2: out of memory for frame conversion\n");
        return 1;
      }
      frame_planes[i][0] = frames[i];
      frame_planes[i][1] = frames[i] + (size_t)w * h;
      frame_planes[i][2] = frames[i] + (size_t)w * h + (size_t)(w / 2) * (h / 2);
      frame_strides[i][0] = w;
      frame_strides[i][1] = w / 2;
      frame_strides[i][2] = w / 2;
    }
    frames_size = need;
  }

  int slot = frame_next;
  frame_next = (frame_next + 1) % NUM_FRAMES;
  frame_index = slot;

  unsigned char* dst_y = frame_planes[slot][0];
  unsigned char* dst_u = frame_planes[slot][1];
  unsigned char* dst_v = frame_planes[slot][2];

  if (stride0 <= 0) stride0 = w;
  if (stride1 <= 0) stride1 = w;

  const unsigned char* src_y = cap_bufs[index].start[0];
  /* Single-planar NV12 keeps the interleaved UV plane right below luma in the
   * same mapping, at the same stride. */
  const unsigned char* src_uv = (cap_planes > 1)
                                    ? cap_bufs[index].start[1]
                                    : src_y + (size_t)stride0 * h;

  if (prof_enabled < 0)
    prof_enabled = getenv("V4L2M2M_PROF") != NULL;

  double t0 = prof_enabled ? prof_now() : 0;
  for (int y = 0; y < h; y++)
    memcpy(dst_y + (size_t)y * w, src_y + (size_t)y * stride0, w);
  if (prof_enabled) {
    prof_y_ms += prof_now() - t0;
    t0 = prof_now();
  }

  int half_w = w / 2, half_h = h / 2;
  for (int y = 0; y < half_h; y++) {
    const unsigned char* src = src_uv + (size_t)y * stride1;
    unsigned char* u = dst_u + (size_t)y * half_w;
    unsigned char* v = dst_v + (size_t)y * half_w;
#ifdef HAVE_NEON
    int x = 0;
    for (; x + 8 <= half_w; x += 8) {
      uint8x8x2_t d = vld2_u8(src + 2 * x);
      vst1_u8(u + x, d.val[0]);
      vst1_u8(v + x, d.val[1]);
    }
    for (; x < half_w; x++) {
      u[x] = src[2 * x];
      v[x] = src[2 * x + 1];
    }
#else
    for (int x = 0; x < half_w; x++) {
      u[x] = src[2 * x];
      v[x] = src[2 * x + 1];
    }
#endif
  }

  if (prof_enabled) {
    prof_uv_ms += prof_now() - t0;
    prof_count++;
    if (prof_count % 30 == 0)
      fprintf(stderr, "[prof] frames=%d avg_y=%.2fms avg_uv=%.2fms\n",
              prof_count, prof_y_ms / prof_count, prof_uv_ms / prof_count);
  }

  return 0;
}

int v4l2m2m_dequeue(int timeout_ms) {
  int index = v4l2m2m_dequeue_raw(timeout_ms);
  if (index <= 0)
    return index;

  int produced = 0;
  if (!last_error_flag)
    produced = convert_frame(index, cap_stride[0], cap_stride[1]) == 0 ? 1 : 0;

  queue_capture(index);
  return produced;
}

unsigned char** v4l2m2m_planes(void) {
  return frame_planes[frame_index];
}

int* v4l2m2m_strides(void) {
  return frame_strides[frame_index];
}

void v4l2m2m_cleanup(void) {
  if (fd >= 0) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    xioctl(fd, VIDIOC_STREAMOFF, &type);
  }

  for (int i = 0; i < cap_count && i < CAP_BUFFERS; i++)
    for (int j = 0; j < cap_planes; j++)
      if (cap_bufs[i].dmabuf[j] >= 0) {
        close(cap_bufs[i].dmabuf[j]);
        cap_bufs[i].dmabuf[j] = -1;
      }

  for (int i = 0; i < out_count && i < OUT_BUFFERS; i++) {
    for (int j = 0; j < MAX_PLANES; j++) {
      if (out_bufs[i].start[j] != NULL && out_bufs[i].start[j] != MAP_FAILED)
        munmap(out_bufs[i].start[j], out_bufs[i].length[j]);
      out_bufs[i].start[j] = NULL;
    }
  }
  for (int i = 0; i < cap_count && i < CAP_BUFFERS; i++) {
    for (int j = 0; j < cap_planes; j++) {
      if (cap_bufs[i].start[j] != NULL && cap_bufs[i].start[j] != MAP_FAILED)
        munmap(cap_bufs[i].start[j], cap_bufs[i].length[j]);
      cap_bufs[i].start[j] = NULL;
    }
  }
  out_count = 0;
  cap_count = 0;
  cap_started = 0;
  gfmt_w = 0;
  gfmt_h = 0;

  if (fd >= 0) {
    close(fd);
    fd = -1;
  }
}
