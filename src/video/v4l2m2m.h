#ifndef V4L2M2M_H
#define V4L2M2M_H

#ifdef __cplusplus
extern "C" {
#endif

/* Probe: is a usable V4L2 M2M multiplanar decoder present? */
int v4l2m2m_available(void);

/* Compressed input formats supported by this backend. */
#define V4L2M2M_CODEC_H264 0
#define V4L2M2M_CODEC_HEVC 1

/* Open the decoder and negotiate NV12M capture at width x height. */
int v4l2m2m_init_codec(int width, int height, int codec);
/* Backwards-compatible H.264 entry point. */
int v4l2m2m_init(int width, int height);

/* Queue one H.264/HEVC access unit (0 on success, -1 on failure). */
int v4l2m2m_feed(const unsigned char* data, int length);

/*
 * Zero-copy API.
 * dequeue_raw() returns a capture buffer index, its pixels untouched and the
 * buffer still owned by the caller. Hand it back with release() as soon as it
 * is no longer being scanned out.
 */
int v4l2m2m_dequeue_raw(int timeout_ms);
int v4l2m2m_release(int index);
int v4l2m2m_had_error(void);

/* DMA-BUF for one plane of a capture buffer (exported on first use). */
int v4l2m2m_dmabuf_fd(int index, int plane);

void* v4l2m2m_plane_ptr(int index, int plane);
int v4l2m2m_plane_len(int index, int plane);
void v4l2m2m_geometry(int* w, int* h, int* stride0, int* stride1);
int v4l2m2m_buffer_count(void);
int v4l2m2m_decoder_fd(void);
/* Planes in the negotiated capture format: 2 for NV12M, 1 for single-plane
 * NV12 (decoders on other SoCs may only expose the latter). */
int v4l2m2m_plane_count(void);
/* /dev/videoN that was probed, for logging. */
const char* v4l2m2m_device_path(void);

/*
 * CPU-copy fallback: dequeue() converts the frame to planar YUV420P and
 * re-queues the buffer immediately. Returns 1 when a frame was produced.
 */
int v4l2m2m_dequeue(int timeout_ms);
unsigned char** v4l2m2m_planes(void);
int* v4l2m2m_strides(void);

void v4l2m2m_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* V4L2M2M_H */
