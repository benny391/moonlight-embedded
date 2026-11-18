#ifndef DRMZERO_H
#define DRMZERO_H

#include "v4l2m2m.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Zero-copy DRM/KMS video output: the decoder's DMA-BUFs are scanned out
 * directly by a hardware overlay plane, so the CPU never touches pixels.
 */

/* Is a usable display plane plus hardware decoder available? */
int drm_video_available(void);

/* Set the KMS mode, initialise the selected decoder and build its FBs. */
int drm_video_init_codec(int width, int height, int codec);
/* Backwards-compatible H.264 entry point. */
int drm_video_init(int width, int height);

/*
 * Feed one H.264/HEVC access unit and display whatever the VPU has finished.
 * Returns the number of frames pushed to the display, or -1 on fatal error.
 */
int drm_video_submit(const unsigned char* data, int length);

void drm_video_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* DRMZERO_H */
