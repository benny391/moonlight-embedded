/*
 * Zero-copy DRM/KMS video output for Moonlight Embedded.
 *
 * The VPU decodes straight into DMA-BUF backed capture buffers, which are
 * imported as DRM framebuffers and scanned out by a hardware overlay plane.
 * No pixel data ever travels through the CPU: the decoder's mmap buffers are
 * mapped uncached, so a CPU copy alone costs ~25ms per 1080p frame.
 *
 * This file is part of Moonlight Embedded.
 *
 * Moonlight is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 */

#include "drmzero.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <signal.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "v4l2m2m.h"

#define MAX_FB 64
/* Connectors on a meson SoC: composite, HDMI and optionally CVBS/component. */
#define MAX_CONN 8
/* Flips kept outstanding; the display retires them one per refresh. Zero
 * selects the overwrite mode: the newest frame is presented, and if the
 * previous flip has not been latched yet the new frame is dropped rather than
 * queued behind it. Queueing is what turns a one-refresh offset into a stable
 * two-refresh offset at 60 fps on a 60 Hz panel, so the default is to skip. */
#define MAX_INFLIGHT_DEFAULT 0
#define MAX_INFLIGHT_LIMIT 8

/* How long a frame that would be overwritten may wait for the previous flip
 * to latch before it is given up on. A frame that arrives a hair before the
 * latch is worth a millisecond or two of patience: presenting it one refresh
 * later keeps motion continuous and costs no extra input latency, because the
 * alternative is not showing it at all. 0 restores drop-immediately. */
#define OVW_WAIT_MS_DEFAULT 2
/* How long the very first frames may block waiting for the VPU. */
#define FIRST_FRAME_WAIT_MS 500
/* HEVC decodes with more latency than H.264; draining on a zero timeout
 * starves the capture queue and the driver stops returning input buffers. */
#define FRAME_WAIT_MS 8

static int dfd = -1;
static int active;

static uint32_t plane_id, crtc_id, conn_id, prim_plane;
static drmModeModeInfo mode;

static uint32_t fb_for_buf[MAX_FB];
static int nbuf;

/* plane property ids */
static uint32_t p_fb, p_crtc, p_sx, p_sy, p_sw, p_sh, p_cx, p_cy, p_cw, p_ch;

/* in-flight capture buffer indices, retired in submission order */
static int inflight[MAX_FB];
static double inflight_t[MAX_FB];   /* when the frame was decoded */
static int icount;
static int max_inflight = MAX_INFLIGHT_DEFAULT;
static int ovw_wait_ms = OVW_WAIT_MS_DEFAULT;

/* Rolling presentation stats, printed with the fps line every 5s. */
static long st_shown, st_drop, st_peak_pending, st_waited;
/* Packets in / frames out per window. pkts == deq proves the client is a pure
 * passthrough and the frame rate is decided by the host, not by the decoder. */
static long st_pkts, st_deq, st_fail;
static long tot_shown, tot_drop;
/* Decoder latency: feed() has taken the packet, how long until a raw frame
 * comes back. This is the only part of the client pipeline a different OS could
 * plausibly change; scanout is pinned to the refresh rate. */
static double st_dec_sum, st_dec_max, tot_dec_sum;
static double tot_lat_sum, tot_lat_max;
static double st_lat_sum, st_lat_max;

static int committed_once;
/* destination rectangle on screen (letterbox aware) */
static int dst_x, dst_y, dst_w, dst_h;

static int vid_w, vid_h;

/* The kernel hands us the vblank timestamp of the flip itself, which is the only
 * honest "when did this frame reach the panel" value: polling later would add
 * however long the fd sat unread. */
static double last_flip_ts;

/* Phase probe.
 *
 * drmCrtcGetSequence reports the last vblank's counter and its timestamp on the
 * same CLOCK_MONOTONIC base the page-flip event uses, so the distance from a
 * commit to the next vblank boundary is measurable instead of guessed from the
 * latency it later shows up as. A frame only makes the coming refresh if the
 * commit lands before the CRTC's internal deadline; the margin at commit time
 * is therefore the number that separates hits from misses, and the split of
 * hits against misses per margin bin is what a phase lock has to be built on.
 */
static int vseq_ok = 1;              /* cleared if the driver says no */
static int have_vseq;
static uint64_t vseq_last;
static double vns_last;
static double period_ms = 16.667;    /* refined from the vblank timestamps */
static long period_n;

/* per-frame probe values, retired together with the frame */
static double inflight_margin[MAX_FB];  /* ms from commit to next vblank */
static double inflight_pred[MAX_FB];    /* predicted latch time (monotonic s) */

static long st_hit, st_miss, st_drop_ovw, st_drop_stale;
static double st_hit_msum, st_hit_mmin, st_miss_msum, st_miss_mmax;
static long st_hit_n, st_miss_n;
static long tot_hit, tot_miss, tot_drop_ovw, tot_drop_stale;

/*
 * Stall watchdog.
 *
 * The one failure nothing else catches: packets keep arriving, the VPU stops
 * producing frames. The process stays alive, systemd sees a healthy unit, and
 * the picture simply freezes forever. Counting packets instead of watching the
 * clock matters: when the host stops sending (paused, PC asleep, static screen)
 * there are no packets either, so that case correctly never fires.
 * SIGINT is deliberate - SIGKILL wedges the VDEC until the next reboot.
 */
static double t_last_frame;     /* when a frame actually reached the panel */
static long pkts_no_frame;      /* packets fed since then */
static int stall_fired;
static double stall_seconds = 6.0;
static long stall_pkts = 30;

static void flip_handler(int fd, unsigned int frame, unsigned int sec,
                         unsigned int usec, void *data) {
  (void)fd; (void)frame; (void)data;
  last_flip_ts = (double)sec + (double)usec / 1e6;
}

static drmEventContext evctx = {
  .version = 2,
  .page_flip_handler = flip_handler,
};

static int wait_flip(int timeout_ms) {
  struct pollfd pfd = { .fd = dfd, .events = POLLIN, .revents = 0 };
  int r = poll(&pfd, 1, timeout_ms);
  if (r <= 0)
    return r;
  return drmHandleEvent(dfd, &evctx) == 0 ? 1 : 0;
}

static double now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* Sample the CRTC's vblank counter. Returns 0, and switches itself off for
 * good, when the driver does not implement GET_SEQUENCE. */
static int vblank_sample(uint64_t *seq, double *vns) {
  if (!vseq_ok)
    return 0;
  struct drm_crtc_get_sequence gs = { .crtc_id = crtc_id };
  if (ioctl(dfd, DRM_IOCTL_CRTC_GET_SEQUENCE, &gs) != 0 ||
      gs.sequence_ns <= 0) {
    vseq_ok = 0;
    fprintf(stderr, "DRM: vblank probe unsupported, phase stats disabled\n");
    return 0;
  }
  *seq = gs.sequence;
  *vns = (double)gs.sequence_ns / 1e9;

  /* Two samples roughly one refresh apart give the real period, which is not
   * necessarily what htotal*vtotal/clock claims. */
  if (have_vseq && *seq > vseq_last && *seq - vseq_last < 4) {
    double p = (*vns - vns_last) * 1000.0 / (double)(*seq - vseq_last);
    if (p > 5.0 && p < 40.0) {
      period_ms += (p - period_ms) * 0.1;
      period_n++;
    }
  }
  have_vseq = 1;
  vseq_last = *seq;
  vns_last = *vns;
  return 1;
}

/* Decode-to-scanout time for one frame, measured when it is retired. */
static void account_flip(void) {
  if (icount <= 0)
    return;
  double ts = last_flip_ts > 0 ? last_flip_ts : now_s();
  double lat = (ts - inflight_t[0]) * 1000.0;
  if (lat > 0) {
    st_lat_sum += lat;
    if (lat > st_lat_max)
      st_lat_max = lat;
    tot_lat_sum += lat;
    if (lat > tot_lat_max)
      tot_lat_max = lat;
  }
  /* Did this commit make the very next refresh, or slip a whole period? The
   * event timestamp is the vblank the frame was latched at, so comparing it
   * with the predicted one answers that without ambiguity. */
  if (inflight_pred[0] > 0) {
    double d = (ts - inflight_pred[0]) * 1000.0;
    if (d < period_ms * 0.5) {
      st_hit++;
      st_hit_n++;
      tot_hit++;
      st_hit_msum += inflight_margin[0];
      if (inflight_margin[0] < st_hit_mmin)
        st_hit_mmin = inflight_margin[0];
    } else {
      st_miss++;
      st_miss_n++;
      tot_miss++;
      st_miss_msum += inflight_margin[0];
      if (inflight_margin[0] > st_miss_mmax)
        st_miss_mmax = inflight_margin[0];
    }
  }
  st_shown++;
  tot_shown++;
}

static void retire_one(void) {
  if (icount <= 0)
    return;
  account_flip();
  v4l2m2m_release(inflight[0]);
  memmove(inflight, inflight + 1, (icount - 1) * sizeof(int));
  memmove(inflight_t, inflight_t + 1, (icount - 1) * sizeof(double));
  memmove(inflight_margin, inflight_margin + 1,
          (icount - 1) * sizeof(double));
  memmove(inflight_pred, inflight_pred + 1, (icount - 1) * sizeof(double));
  icount--;
}

static void retire_all(void) {
  while (icount > 0) {
    if (wait_flip(100) <= 0)
      break;
    retire_one();
  }
}

static uint32_t prop_id(drmModeObjectProperties *props, const char *name) {
  for (uint32_t i = 0; i < props->count_props; i++) {
    drmModePropertyRes *prp = drmModeGetProperty(dfd, props->props[i]);
    if (prp == NULL)
      continue;
    int hit = strcmp(prp->name, name) == 0;
    uint32_t id = prp->prop_id;
    drmModeFreeProperty(prp);
    if (hit)
      return id;
  }
  return 0;
}

static void compute_letterbox(void) {
  double ar_v = (double)vid_w / vid_h;
  double ar_d = (double)mode.hdisplay / mode.vdisplay;
  if (ar_v > ar_d) {
    dst_w = mode.hdisplay;
    dst_h = (int)(mode.hdisplay / ar_v + 0.5);
  } else {
    dst_h = mode.vdisplay;
    dst_w = (int)(mode.vdisplay * ar_v + 0.5);
  }
  dst_x = (mode.hdisplay - dst_w) / 2;
  dst_y = (mode.vdisplay - dst_h) / 2;
}

/* The card node is not fixed either: /dev/dri/card0 is the usual first GPU
 * but SoCs with a separate render node or an extra display controller can put
 * the KMS device elsewhere. Probe the common nodes in order and remember the
 * one that worked. */
static char card_path[32] = "/dev/dri/card0";

static int open_card(void) {
  char path[32];
  for (int i = 0; i < 16; i++) {
    snprintf(path, sizeof(path), "/dev/dri/card%d", i);
    if (access(path, R_OK | W_OK) != 0)
      continue;
    int f = open(path, O_RDWR);
    if (f < 0)
      continue;
    /* Skip the GPU node: it answers GetResources but has no connectors, and
     * a KMS scanout without a CRTC is useless here. */
    drmModeRes* res = drmModeGetResources(f);
    int usable = res != NULL && res->count_crtcs > 0 && res->count_connectors > 0;
    if (res != NULL)
      drmModeFreeResources(res);
    if (!usable) {
      close(f);
      continue;
    }
    snprintf(card_path, sizeof(card_path), "%s", path);
    return f;
  }
  return -1;
}

int drm_video_available(void) {
  const char *env = getenv("MOONLIGHT_FORCE_SW");
  if (env != NULL && env[0] != '\0' && strcmp(env, "0") != 0)
    return 0;
  if (!v4l2m2m_available())
    return 0;

  int f = open_card();
  if (f < 0)
    return 0;
  drmSetClientCap(f, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
  drmSetClientCap(f, DRM_CLIENT_CAP_ATOMIC, 1);

  int ok = 0;
  drmModePlaneRes *pr = drmModeGetPlaneResources(f);
  for (unsigned i = 0; pr && i < pr->count_planes && !ok; i++) {
    drmModePlane *p = drmModeGetPlane(f, pr->planes[i]);
    if (p == NULL)
      continue;
    for (unsigned j = 0; j < p->count_formats; j++)
      if (p->formats[j] == DRM_FORMAT_NV12) {
        ok = 1;
        break;
      }
    drmModeFreePlane(p);
  }
  if (pr)
    drmModeFreePlaneResources(pr);
  close(f);
  return ok;
}

/*
 * Pick the display mode for a connector. Preference order:
 *   1. exactly the stream resolution, any refresh rate;
 *   2. otherwise the largest non-interlaced mode at >= 50Hz that is no wider
 *      than the stream (capped at 1920 when the stream is smaller), so the
 *      panel's own scaler handles the upscale.
 *
 * The 50Hz floor is deliberately hard: a 4K30 "PREFERRED" EDID entry used to
 * win on area alone, which halves the frame rate and starves the VDEC output
 * queue ("Video decode unit queue overflow"). Refresh rate beats pixel count
 * for game streaming.
 */
/*
 * An explicit output mode wins over the heuristics below. That is how a 2K
 * stream reaches a 4K panel: the VPU scaler does the single upscale, instead of
 * the box downscaling to 1080p and the TV scaling back up again.
 * Format: MOONLIGHT_OUTPUT_MODE=3840x2160@60 ("@" optional).
 */
static int output_override(int *w, int *h, int *refresh) {
  const char *env = getenv("MOONLIGHT_OUTPUT_MODE");
  int ow = 0, oh = 0, orr = 0;

  *w = *h = *refresh = 0;
  if (env == NULL || sscanf(env, "%dx%d@%d", &ow, &oh, &orr) < 2)
    return 0;
  if (ow <= 0 || oh <= 0)
    return 0;
  *w = ow;
  *h = oh;
  *refresh = orr > 0 ? orr : 0;
  return 1;
}

/* Built-in CTA-861 timings for the sizes we care about.  A display that was
 * asleep at boot leaves the connector with nothing but the VESA fallback list,
 * so the requested output size is usually missing there; handing the CRTC our
 * own timing keeps the picture at the panel's real resolution instead of
 * silently downgrading to whatever happens to be listed. */
static int std_timing(int w, int h, drmModeModeInfo *m) {
  memset(m, 0, sizeof(*m));
  if (w == 3840 && h == 2160) {
    m->clock = 594000;
    m->hdisplay = 3840; m->hsync_start = 4016; m->hsync_end = 4104;
    m->htotal = 4400;
    m->vdisplay = 2160; m->vsync_start = 2168; m->vsync_end = 2178;
    m->vtotal = 2250;
  } else if (w == 2560 && h == 1440) {
    m->clock = 241500;
    m->hdisplay = 2560; m->hsync_start = 2608; m->hsync_end = 2640;
    m->htotal = 2720;
    m->vdisplay = 1440; m->vsync_start = 1443; m->vsync_end = 1448;
    m->vtotal = 1481;
  } else if (w == 1920 && h == 1080) {
    m->clock = 148500;
    m->hdisplay = 1920; m->hsync_start = 2008; m->hsync_end = 2052;
    m->htotal = 2200;
    m->vdisplay = 1080; m->vsync_start = 1084; m->vsync_end = 1089;
    m->vtotal = 1125;
  } else {
    return 0;
  }
  m->vrefresh = 60;
  m->flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC;
  m->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
  snprintf(m->name, sizeof(m->name), "%dx%d", w, h);
  return 1;
}

static int find_override_mode(drmModeConnector *c, int w, int h, int refresh) {
  for (int m = 0; m < c->count_modes; m++) {
    drmModeModeInfo *mi = &c->modes[m];
    if (mi->flags & DRM_MODE_FLAG_INTERLACE)
      continue;
    if (mi->hdisplay != w || mi->vdisplay != h)
      continue;
    if (refresh > 0 && abs(mi->vrefresh - refresh) > 1)
      continue;
    return m;
  }
  return -1;
}

static int pick_mode(drmModeConnector *c, int want_w, int want_h) {
  int cap_w = want_w > 1920 ? want_w : 1920;
  int best = -1;
  long best_score = -1;

  for (int m = 0; m < c->count_modes; m++) {
    drmModeModeInfo *mi = &c->modes[m];
    if (mi->hdisplay <= 0 || mi->vdisplay <= 0)
      continue;
    if (mi->flags & DRM_MODE_FLAG_INTERLACE)
      continue;

    int refresh = mi->vrefresh ? mi->vrefresh : 60;
    long score;

    if (want_w > 0 && mi->hdisplay == want_w && mi->vdisplay == want_h) {
      score = 1000000L + refresh;
    } else if (refresh >= 50 && mi->hdisplay <= cap_w) {
      score = (long)mi->hdisplay * mi->vdisplay / 1000 + refresh;
      if (mi->type & DRM_MODE_TYPE_PREFERRED)
        score += 5000;
    } else {
      continue;
    }

    if (score > best_score) {
      best_score = score;
      best = m;
    }
  }

  /* Last resort: the sink only offers sub-50Hz or oversized modes. */
  if (best < 0) {
    for (int m = 0; m < c->count_modes; m++) {
      drmModeModeInfo *mi = &c->modes[m];
      if (mi->hdisplay <= 0 || mi->vdisplay <= 0)
        continue;
      if (mi->flags & DRM_MODE_FLAG_INTERLACE)
        continue;
      long score = (long)mi->hdisplay * mi->vdisplay / 1000;
      if (score > best_score) {
        best_score = score;
        best = m;
      }
    }
  }
  return best;
}

/* Connectors other than the one we drive. meson-drm has a single CRTC shared
 * by every connector, so they must be detached explicitly or the atomic
 * commit trips over "can't enable cloning". */
static uint32_t other_conn[MAX_CONN];
static uint32_t other_conn_crtc[MAX_CONN];
static int n_other_conn;

static int setup_kms(void) {
  dfd = open_card();
  if (dfd < 0) {
    perror("drm: open card");
    return -1;
  }
  printf("DRM: using %s\n", card_path);
  drmSetClientCap(dfd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
  drmSetClientCap(dfd, DRM_CLIENT_CAP_ATOMIC, 1);

  drmModeRes *res = drmModeGetResources(dfd);
  if (res == NULL) {
    fprintf(stderr, "drm: no resources\n");
    return -1;
  }

  int best_score = -1;
  int ov_w = 0, ov_h = 0, ov_refresh = 0;
  crtc_id = 0;
  n_other_conn = 0;

  output_override(&ov_w, &ov_h, &ov_refresh);
  if (ov_w > 0)
    fprintf(stderr, "DRM: output mode request %dx%d@%d\n", ov_w, ov_h,
            ov_refresh);

  for (int i = 0; i < res->count_connectors; i++) {
    drmModeConnector *c = drmModeGetConnector(dfd, res->connectors[i]);
    if (c == NULL)
      continue;

    if (c->connection == DRM_MODE_CONNECTED && c->count_modes > 0) {
      /* A digital output always beats composite: CVBS is 720x576i@50, which
       * both looks terrible and caps the achievable frame rate at 50Hz. */
      int type_score;
      switch (c->connector_type) {
      case DRM_MODE_CONNECTOR_HDMIA:
      case DRM_MODE_CONNECTOR_HDMIB:
      case DRM_MODE_CONNECTOR_DisplayPort:
        type_score = 3000;
        break;
      case DRM_MODE_CONNECTOR_DVID:
      case DRM_MODE_CONNECTOR_DVII:
        type_score = 2000;
        break;
      case DRM_MODE_CONNECTOR_Component:
        type_score = 500;
        break;
      default:
        type_score = 100;
        break;
      }
      int score = type_score + c->count_modes;

      int pick = -1;
      int synth = 0;
      drmModeModeInfo synth_mode;
      if (ov_w > 0)
        pick = find_override_mode(c, ov_w, ov_h, ov_refresh);
      if (pick < 0 && ov_w > 0)
        synth = std_timing(ov_w, ov_h, &synth_mode);
      if (pick < 0 && !synth)
        pick = pick_mode(c, vid_w, vid_h);
      if ((pick >= 0 || synth) && score > best_score) {
        best_score = score;
        conn_id = c->connector_id;
        mode = synth ? synth_mode : c->modes[pick];

        crtc_id = 0;
        if (c->encoder_id) {
          drmModeEncoder *e = drmModeGetEncoder(dfd, c->encoder_id);
          if (e) {
            crtc_id = e->crtc_id;
            drmModeFreeEncoder(e);
          }
        }
        if (!crtc_id) {
          /* Not bound yet (fresh HPD): take any CRTC an encoder says it can
           * route to this connector type. */
          for (int ei = 0; ei < res->count_encoders && !crtc_id; ei++) {
            drmModeEncoder *e = drmModeGetEncoder(dfd, res->encoders[ei]);
            if (e == NULL)
              continue;
            for (int ci = 0; ci < res->count_crtcs; ci++)
              if (e->possible_crtcs & (1 << ci)) {
                crtc_id = res->crtcs[ci];
                break;
              }
            drmModeFreeEncoder(e);
          }
        }
      }
    }
    drmModeFreeConnector(c);
  }

  if (best_score < 0 || crtc_id == 0) {
    fprintf(stderr, "drm: no connected display\n");
    return -1;
  }
  printf("DRM: connector %u, mode %dx%d@%d\n", conn_id, mode.hdisplay,
         mode.vdisplay, mode.vrefresh);

  /* Seed the phase probe with the mode's nominal period (clock is in kHz, so
   * htotal*vtotal/clock is already milliseconds); the vblank timestamps refine
   * it at runtime. */
  if (mode.clock > 0 && mode.htotal > 0 && mode.vtotal > 0)
    period_ms = (double)((long)mode.htotal * (long)mode.vtotal) /
                (double)mode.clock;
  st_hit_mmin = 1e9;

  /* Remember every other connector that is still bound to a CRTC so the first
   * atomic commit can detach it. */
  for (int i = 0; i < res->count_connectors && n_other_conn < MAX_CONN; i++) {
    if ((int)res->connectors[i] == (int)conn_id)
      continue;
    drmModeObjectProperties *op = drmModeObjectGetProperties(
        dfd, res->connectors[i], DRM_MODE_OBJECT_CONNECTOR);
    if (op == NULL)
      continue;
    for (unsigned k = 0; k < op->count_props; k++) {
      drmModePropertyRes *p = drmModeGetProperty(dfd, op->props[k]);
      if (p == NULL)
        continue;
      int match = strcmp(p->name, "CRTC_ID") == 0;
      drmModeFreeProperty(p);
      if (match) {
        other_conn[n_other_conn] = res->connectors[i];
        other_conn_crtc[n_other_conn] = op->props[k];
        n_other_conn++;
        break;
      }
    }
    drmModeFreeObjectProperties(op);
  }

  /* pick planes: the NV12 one for video, an XRGB one for the black backdrop */
  plane_id = prim_plane = 0;
  drmModePlaneRes *pr = drmModeGetPlaneResources(dfd);
  for (unsigned i = 0; pr && i < pr->count_planes; i++) {
    drmModePlane *p = drmModeGetPlane(dfd, pr->planes[i]);
    if (p == NULL)
      continue;
    int has_nv12 = 0, has_xrgb = 0;
    for (unsigned j = 0; j < p->count_formats; j++) {
      if (p->formats[j] == DRM_FORMAT_NV12)
        has_nv12 = 1;
      if (p->formats[j] == DRM_FORMAT_XRGB8888)
        has_xrgb = 1;
    }
    if (has_nv12 && !plane_id)
      plane_id = p->plane_id;
    else if (has_xrgb && !prim_plane)
      prim_plane = p->plane_id;
    drmModeFreePlane(p);
  }
  if (pr)
    drmModeFreePlaneResources(pr);
  if (!plane_id || !prim_plane) {
    fprintf(stderr, "drm: no usable planes (nv12=%u xrgb=%u)\n", plane_id, prim_plane);
    return -1;
  }

  /* black dumb buffer to clear everything the video plane does not cover */
  struct drm_mode_create_dumb cd;
  memset(&cd, 0, sizeof(cd));
  cd.width = mode.hdisplay;
  cd.height = mode.vdisplay;
  cd.bpp = 32;
  if (drmIoctl(dfd, DRM_IOCTL_MODE_CREATE_DUMB, &cd) < 0) {
    perror("drm: CREATE_DUMB");
    return -1;
  }
  uint32_t prim_fb = 0;
  if (drmModeAddFB(dfd, mode.hdisplay, mode.vdisplay, 24, 32, cd.pitch,
                   cd.handle, &prim_fb) != 0) {
    perror("drm: AddFB primary");
    return -1;
  }
  struct drm_mode_map_dumb md;
  memset(&md, 0, sizeof(md));
  md.handle = cd.handle;
  if (drmIoctl(dfd, DRM_IOCTL_MODE_MAP_DUMB, &md) == 0) {
    void *map = mmap(0, cd.size, PROT_READ | PROT_WRITE, MAP_SHARED, dfd, md.offset);
    if (map != MAP_FAILED) {
      memset(map, 0, cd.size);
      munmap(map, cd.size);
    }
  }

  drmModeObjectProperties *props;
  uint32_t conn_crtc, crtc_mode, crtc_active, prim_crtc, prim_fb_id,
           prim_sw, prim_sh, prim_cw, prim_ch, prim_zpos, p_zpos;

  props = drmModeObjectGetProperties(dfd, conn_id, DRM_MODE_OBJECT_CONNECTOR);
  conn_crtc = props ? prop_id(props, "CRTC_ID") : 0;
  if (props) drmModeFreeObjectProperties(props);

  props = drmModeObjectGetProperties(dfd, crtc_id, DRM_MODE_OBJECT_CRTC);
  crtc_mode = props ? prop_id(props, "MODE_ID") : 0;
  crtc_active = props ? prop_id(props, "ACTIVE") : 0;
  if (props) drmModeFreeObjectProperties(props);

  props = drmModeObjectGetProperties(dfd, prim_plane, DRM_MODE_OBJECT_PLANE);
  prim_crtc = props ? prop_id(props, "CRTC_ID") : 0;
  prim_fb_id = props ? prop_id(props, "FB_ID") : 0;
  prim_sw = props ? prop_id(props, "SRC_W") : 0;
  prim_sh = props ? prop_id(props, "SRC_H") : 0;
  prim_cw = props ? prop_id(props, "CRTC_W") : 0;
  prim_ch = props ? prop_id(props, "CRTC_H") : 0;
  prim_zpos = props ? prop_id(props, "zpos") : 0;
  if (props) drmModeFreeObjectProperties(props);

  props = drmModeObjectGetProperties(dfd, plane_id, DRM_MODE_OBJECT_PLANE);
  p_fb = props ? prop_id(props, "FB_ID") : 0;
  p_crtc = props ? prop_id(props, "CRTC_ID") : 0;
  p_sx = props ? prop_id(props, "SRC_X") : 0;
  p_sy = props ? prop_id(props, "SRC_Y") : 0;
  p_sw = props ? prop_id(props, "SRC_W") : 0;
  p_sh = props ? prop_id(props, "SRC_H") : 0;
  p_cx = props ? prop_id(props, "CRTC_X") : 0;
  p_cy = props ? prop_id(props, "CRTC_Y") : 0;
  p_cw = props ? prop_id(props, "CRTC_W") : 0;
  p_ch = props ? prop_id(props, "CRTC_H") : 0;
  p_zpos = props ? prop_id(props, "zpos") : 0;
  if (props) drmModeFreeObjectProperties(props);

  if (!conn_crtc || !crtc_mode || !p_fb || !p_crtc) {
    fprintf(stderr, "drm: missing required plane properties\n");
    return -1;
  }

  uint32_t blob = 0;
  if (drmModeCreatePropertyBlob(dfd, &mode, sizeof(mode), &blob) != 0) {
    perror("drm: CreatePropertyBlob");
    return -1;
  }

  /* meson stacks the video (NV12) plane *below* the OSD (RGB) plane, and both
   * are full screen here, so an opaque backdrop would hide the video entirely.
   * Commit an explicit z-order first; if the driver refuses to reorder, fall
   * back to switching the backdrop plane off and showing video alone. */
  int rc = -1;
  int used_backdrop = 0;

  for (int attempt = 0; attempt < 2 && rc != 0; attempt++) {
    int backdrop = (attempt == 0);
    drmModeAtomicReq *r = drmModeAtomicAlloc();

    /* Detach the connectors we are not driving; the meson CRTC is shared, and a
     * leftover binding shows up as "can't enable cloning" in dmesg. */
    for (int i = 0; i < n_other_conn; i++)
      drmModeAtomicAddProperty(r, other_conn[i], other_conn_crtc[i], 0);
    drmModeAtomicAddProperty(r, conn_id, conn_crtc, crtc_id);
    drmModeAtomicAddProperty(r, crtc_id, crtc_mode, blob);
    drmModeAtomicAddProperty(r, crtc_id, crtc_active, 1);

    if (backdrop) {
      drmModeAtomicAddProperty(r, prim_plane, prim_crtc, crtc_id);
      drmModeAtomicAddProperty(r, prim_plane, prim_fb_id, prim_fb);
      if (prim_sw)
        drmModeAtomicAddProperty(r, prim_plane, prim_sw, mode.hdisplay << 16);
      if (prim_sh)
        drmModeAtomicAddProperty(r, prim_plane, prim_sh, mode.vdisplay << 16);
      if (prim_cw)
        drmModeAtomicAddProperty(r, prim_plane, prim_cw, mode.hdisplay);
      if (prim_ch)
        drmModeAtomicAddProperty(r, prim_plane, prim_ch, mode.vdisplay);
      if (prim_zpos)
        drmModeAtomicAddProperty(r, prim_plane, prim_zpos, 0);
      if (p_zpos)
        drmModeAtomicAddProperty(r, plane_id, p_zpos, 1);
    } else {
      /* No backdrop: the video plane is the only layer on this CRTC. */
      drmModeAtomicAddProperty(r, prim_plane, prim_crtc, 0);
      drmModeAtomicAddProperty(r, prim_plane, prim_fb_id, 0);
    }

    rc = drmModeAtomicCommit(dfd, r, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
    drmModeAtomicFree(r);
    if (rc == 0)
      used_backdrop = backdrop;
  }

  drmModeDestroyPropertyBlob(dfd, blob);
  if (rc != 0) {
    perror("drm: modeset");
    return -1;
  }
  printf("DRM: layers - %s\n",
         used_backdrop ? "video above black backdrop"
                       : "backdrop plane off, video only");
  return 0;
}

int drm_video_init_codec(int width, int height, int codec) {
  drm_video_cleanup();
  vid_w = width;
  vid_h = height;

  if (setup_kms() < 0)
    goto fail;
  if (v4l2m2m_init_codec(width, height, codec) < 0) {
    fprintf(stderr, "drm: decoder init failed\n");
    goto fail;
  }

  compute_letterbox();
  nbuf = 0; /* framebuffers are created lazily, see ensure_fbs() */
  active = 1;
  committed_once = 0;
  icount = 0;
  t_last_frame = now_s();
  pkts_no_frame = 0;
  stall_fired = 0;
  {
    const char *q = getenv("MOONLIGHT_STALL_SECONDS");
    if (q != NULL)
      stall_seconds = atof(q);
    q = getenv("MOONLIGHT_STALL_PACKETS");
    if (q != NULL)
      stall_pkts = atol(q);
    if (stall_pkts < 1)
      stall_pkts = 1;
  }
  {
    const char *q = getenv("MOONLIGHT_MAX_INFLIGHT");
    int v = q ? atoi(q) : MAX_INFLIGHT_DEFAULT;
    if (v < 0)
      v = 0;
    if (v > MAX_INFLIGHT_LIMIT)
      v = MAX_INFLIGHT_LIMIT;
    max_inflight = v;
  }
  {
    const char *q = getenv("MOONLIGHT_OVW_WAIT_MS");
    int v = q ? atoi(q) : OVW_WAIT_MS_DEFAULT;
    if (v < 0)
      v = 0;
    if (v > 16)
      v = 16;
    ovw_wait_ms = v;
  }
  printf("DRM: zero-copy %dx%d -> %dx%d at %d,%d (framebuffers lazy)\n",
         vid_w, vid_h, dst_w, dst_h, dst_x, dst_y);
  return 0;

fail:
  drm_video_cleanup();
  return -1;
}

int drm_video_init(int width, int height) {
  return drm_video_init_codec(width, height, V4L2M2M_CODEC_H264);
}

/* The decoder's capture buffers only exist after the first packet has been
 * fed, so the DRM framebuffer set must be built on the first decoded frame. */
static int ensure_fbs(void) {
  if (nbuf > 0)
    return 0;

  int sw, sh, s0, s1;
  v4l2m2m_geometry(&sw, &sh, &s0, &s1);
  nbuf = v4l2m2m_buffer_count();
  if (nbuf > MAX_FB)
    nbuf = MAX_FB;
  if (nbuf <= 0) {
    fprintf(stderr, "drm: decoder reported no capture buffers\n");
    return -1;
  }

  /* NV12M gives one DMA-BUF per plane; single-planar NV12 gives one for the
   * whole frame, which is also what DRM_FORMAT_NV12 expects in that case. */
  int nplanes = v4l2m2m_plane_count();
  if (nplanes < 1 || nplanes > 2) {
    fprintf(stderr, "drm: unexpected plane count %d\n", nplanes);
    return -1;
  }

  for (int i = 0; i < nbuf; i++) {
    /* libdrm's AddFB2 signature always expects four entries, even for a
     * single-plane format; only the first `nplanes` are used. */
    uint32_t handles[4] = { 0 };
    for (int p = 0; p < nplanes; p++) {
      int f = v4l2m2m_dmabuf_fd(i, p);
      if (f < 0) {
        fprintf(stderr, "drm: dmabuf export failed for buffer %d plane %d\n", i, p);
        return -1;
      }
      if (drmPrimeFDToHandle(dfd, f, &handles[p]) != 0) {
        fprintf(stderr, "drm: PRIME import failed for buffer %d plane %d\n", i, p);
        return -1;
      }
    }
    uint32_t pitches[4] = { (uint32_t)(s0 > 0 ? s0 : sw),
                            (uint32_t)(nplanes > 1 && s1 > 0 ? s1 : sw), 0, 0 };
    uint32_t offsets[4] = { 0, 0, 0, 0 };
    if (drmModeAddFB2(dfd, sw, sh, DRM_FORMAT_NV12, handles, pitches, offsets,
                      &fb_for_buf[i], 0) != 0) {
      fprintf(stderr, "drm: AddFB2 NV12 failed for buffer %d\n", i);
      return -1;
    }
  }
  printf("DRM: built %d framebuffers (%dx%d stride %d/%d)\n",
         nbuf, sw, sh, s0, s1);
  return 0;
}

int drm_video_submit(const unsigned char *data, int length) {
  if (!active)
    return -1;

  st_pkts++;

  /* Collect any flip that finished while we were idle. Without this the event
   * sits in the DRM fd until the next packet arrives, which both delays buffer
   * reuse and inflates the measured latency by a whole packet interval. */
  while (icount > 0 && wait_flip(0) > 0)
    retire_one();

  pkts_no_frame++;
  if (!stall_fired && stall_seconds > 0 && pkts_no_frame >= stall_pkts &&
      now_s() - t_last_frame > stall_seconds) {
    stall_fired = 1;
    fprintf(stderr,
            "DRM: decoder stalled - %ld packets with no frame for %.1fs, "
            "restarting the stream\n",
            pkts_no_frame, now_s() - t_last_frame);
    /* Process-directed, not raise(): loop.c blocks SIGINT in every thread and
     * collects it through a signalfd in the main thread, which never sees a
     * thread-directed signal. raise() would leave the picture frozen. */
    kill(getpid(), SIGINT);
    return -1;
  }

  double t_feed = now_s();
  if (v4l2m2m_feed(data, length) < 0)
    return -1;

  int shown = 0;
  double newest_t = 0;
  int newest = -1;

  for (;;) {
    /* Once a frame is in hand there is nothing to wait for: the only reason to
     * look again is to discard a second, staler frame, and that either comes
     * back immediately or not at all. Blocking 8 ms to discover it did not
     * arrive delayed every single commit by 8 ms (measured as a constant
     * lat - mhit = 8.1 ms). Waiting is only needed while no frame exists yet:
     * half a second for the very first one, one frame time in steady state. */
    int wait_ms = newest > 0 ? 0
                 : (committed_once ? FRAME_WAIT_MS : FIRST_FRAME_WAIT_MS);
    int idx = v4l2m2m_dequeue_raw(wait_ms);
    if (idx <= 0)
      break;
    st_deq++;
    if (shown == 0 && newest < 0) {
      double dec = (now_s() - t_feed) * 1000.0;
      if (dec > 0) {
        st_dec_sum += dec;
        tot_dec_sum += dec;
        if (dec > st_dec_max)
          st_dec_max = dec;
      }
    }
    if (v4l2m2m_had_error()) {
      v4l2m2m_release(idx);
      continue;
    }
    /* Frames that piled up while the display was busy are already stale: drop
     * them and present only the freshest one, so a burst costs quality of
     * motion rather than a queue of old pictures. */
    if (newest > 0) {
      v4l2m2m_release(newest);
      st_drop++;
      st_drop_stale++;
      tot_drop++;
      tot_drop_stale++;
    }
    newest = idx;
    newest_t = now_s();
  }

  if (newest > 0) {
    if (ensure_fbs() < 0) {
      v4l2m2m_release(newest);
      return -1;
    }

    /* max_inflight == 0 is the overwrite mode: never queue behind a flip that
     * has not been latched yet. Queueing costs a whole extra refresh per entry
     * and at 60 fps on a 60 Hz panel never drains again, so the frame is
     * dropped instead and the pipeline stays one deep. */
    if (max_inflight == 0) {
      while (icount > 0 && wait_flip(0) > 0)
        retire_one();
      if (icount > 0 && ovw_wait_ms > 0) {
        /* The previous flip is still armed, so this frame would be thrown
         * away. Give that flip a bounded chance to latch first: the frame is
         * then presented one refresh later instead of vanishing. */
        if (wait_flip(ovw_wait_ms) > 0)
          retire_one();
        while (icount > 0 && wait_flip(0) > 0)
          retire_one();
      }
      if (icount > 0) {
        st_drop++;
        st_drop_ovw++;
        tot_drop++;
        tot_drop_ovw++;
        v4l2m2m_release(newest);
        goto stats;
      }
    } else {
      while (icount >= max_inflight) {
        st_waited++;
        if (wait_flip(100) <= 0)
          break;
        retire_one();
      }
    }
    if (icount > st_peak_pending)
      st_peak_pending = icount;

    int ok = 0;
    for (int attempt = 0; attempt < 3 && !ok; attempt++) {
      drmModeAtomicReq *r = drmModeAtomicAlloc();
      drmModeAtomicAddProperty(r, plane_id, p_crtc, crtc_id);
      drmModeAtomicAddProperty(r, plane_id, p_fb, fb_for_buf[newest]);
      drmModeAtomicAddProperty(r, plane_id, p_sx, 0);
      drmModeAtomicAddProperty(r, plane_id, p_sy, 0);
      drmModeAtomicAddProperty(r, plane_id, p_sw, (uint32_t)vid_w << 16);
      drmModeAtomicAddProperty(r, plane_id, p_sh, (uint32_t)vid_h << 16);
      drmModeAtomicAddProperty(r, plane_id, p_cx, (uint32_t)dst_x);
      drmModeAtomicAddProperty(r, plane_id, p_cy, (uint32_t)dst_y);
      drmModeAtomicAddProperty(r, plane_id, p_cw, (uint32_t)dst_w);
      drmModeAtomicAddProperty(r, plane_id, p_ch, (uint32_t)dst_h);

      uint32_t flags = DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT;
      if (!committed_once)
        flags |= DRM_MODE_ATOMIC_ALLOW_MODESET;

      /* How long is there left before the next vblank boundary? Negative
       * margins cannot happen, but a margin inside the CRTC deadline is what
       * makes a frame slip a whole period. */
      double t_commit = now_s();
      double pred = 0.0, margin = 0.0;
      uint64_t vseq;
      double vns;
      if (vblank_sample(&vseq, &vns)) {
        pred = vns + period_ms / 1000.0;
        margin = (pred - t_commit) * 1000.0;
      }

      int rc = drmModeAtomicCommit(dfd, r, flags, NULL);
      drmModeAtomicFree(r);

      if (rc == 0) {
        ok = 1;
        committed_once = 1;
        shown++;
        t_last_frame = now_s();
        pkts_no_frame = 0;
        if (icount < MAX_FB) {
          inflight[icount] = newest;
          inflight_t[icount] = newest_t;
          inflight_margin[icount] = margin;
          inflight_pred[icount] = pred;
          icount++;
        } else {
          v4l2m2m_release(newest);
        }
      } else {
        /* the display has not retired the previous flip yet */
        st_waited++;
        st_fail++;
        if (wait_flip(60) > 0)
          retire_one();
      }
    }
    if (!ok) {
      st_fail++;
      v4l2m2m_release(newest);
    }
  }

stats:
  ;
  /* A rolling 5s frame count is the only honest way to tell whether the box is
   * really presenting at the stream's rate after a mode change. */
  static time_t fps_t0;
  static long fps_n;
  time_t now = time(NULL);

  fps_n += shown;
  if (fps_t0 == 0)
    fps_t0 = now;
  else if (now - fps_t0 >= 15) {
    long retired = st_shown > 0 ? st_shown : 1;
    fprintf(stderr,
            "DRM: %.1f fps at %dx%d pending<=%ld/%d drop=%ld(ovw %ld/stale %ld) "
            "lat=%.1fms max=%.1fms dec=%.1f/%.1fms pkts=%ld deq=%ld fail=%ld "
            "per=%.2f hit=%ld miss=%ld mhit=%.1f/%.1f mmiss=%.1f/%.1f\n",
            (double)fps_n / (double)(now - fps_t0), mode.hdisplay,
            mode.vdisplay, st_peak_pending, max_inflight, st_drop,
            st_drop_ovw, st_drop_stale,
            st_lat_sum / (double)retired, st_lat_max,
            st_dec_sum / (double)retired, st_dec_max, st_pkts, st_deq, st_fail,
            period_ms, st_hit, st_miss,
            st_hit_n ? st_hit_msum / (double)st_hit_n : 0.0,
            st_hit_n ? st_hit_mmin : 0.0,
            st_miss_n ? st_miss_msum / (double)st_miss_n : 0.0,
            st_miss_mmax);
    fps_t0 = now;
    fps_n = 0;
    st_shown = st_drop = st_drop_ovw = st_drop_stale = 0;
    st_peak_pending = st_waited = 0;
    st_pkts = st_deq = st_fail = 0;
    st_lat_sum = st_lat_max = 0;
    st_dec_sum = st_dec_max = 0;
    st_hit = st_miss = st_hit_n = st_miss_n = 0;
    st_hit_msum = st_miss_msum = st_miss_mmax = 0;
    st_hit_mmin = 1e9;
  }

  return shown;
}

void drm_video_cleanup(void) {
  if (active) {
    retire_all();
    v4l2m2m_cleanup();
    active = 0;
  }
  if (dfd >= 0) {
    close(dfd);
    dfd = -1;
  }
  committed_once = 0;
  icount = 0;

  if (tot_shown > 0)
    fprintf(stderr,
            "DRM: presented %ld frames, dropped %ld (overwrite %ld, stale %ld), "
            "decode->scanout avg %.1fms max %.1fms, decoder avg %.1fms "
            "(queue %d, hit %ld miss %ld)\n",
            tot_shown, tot_drop, tot_drop_ovw, tot_drop_stale,
            tot_lat_sum / (double)tot_shown, tot_lat_max,
            tot_dec_sum / (double)tot_shown, max_inflight, tot_hit, tot_miss);
  tot_shown = tot_drop = tot_hit = tot_miss = 0;
  tot_drop_ovw = tot_drop_stale = 0;
  tot_lat_sum = tot_lat_max = tot_dec_sum = 0;
}
