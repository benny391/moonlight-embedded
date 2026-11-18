/*
 * This file is part of Moonlight Embedded.
 *
 * Copyright (C) 2015 Iwan Timmer
 *
 * Moonlight is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Moonlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Moonlight; if not, see <http://www.gnu.org/licenses/>.
 */

#include "video.h"
#include "ffmpeg.h"
#include "v4l2m2m.h"
#include "drmzero.h"

#include "../sdl.h"
#include "../util.h"

#include <SDL.h>
#include <SDL_thread.h>

#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define SLICES_PER_FRAME 4
// How long we are willing to wait for the VPU to hand back a decoded frame
#define HW_DECODE_WAIT_MS 8

typedef enum {
  BACKEND_NONE = 0,
  BACKEND_DRM,        // zero copy: decoder DMA-BUF scanned out by a plane
  BACKEND_V4L2_COPY,  // hardware decode, CPU converts to planar YUV for SDL
  BACKEND_FFMPEG,     // software
} backend_t;

static void* decode_buffer;
static size_t decode_buffer_size;
static backend_t backend = BACKEND_NONE;

static int sdl_setup(int videoFormat, int width, int height, int redrawRate, void* context, int drFlags) {
  backend = BACKEND_NONE;

  ensure_buf_size(&decode_buffer, &decode_buffer_size,
                  INITIAL_DECODER_BUFFER_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);

  int hw_codec = -1;
  const char* hw_codec_name = NULL;
  if (videoFormat & VIDEO_FORMAT_MASK_H264) {
    hw_codec = V4L2M2M_CODEC_H264;
    hw_codec_name = "H.264";
  } else if (videoFormat & VIDEO_FORMAT_MASK_H265) {
    hw_codec = V4L2M2M_CODEC_HEVC;
    hw_codec_name = "HEVC";
  }

  if (hw_codec >= 0) {
    // Preferred: decode straight into buffers the display can scan out
    if (drm_video_available()) {
      if (drm_video_init_codec(width, height, hw_codec) == 0) {
        backend = BACKEND_DRM;
        printf("Using hardware video decoder: %s V4L2 M2M + DRM zero-copy (%dx%d)\n",
               hw_codec_name, width, height);
        return 0;
      }
      fprintf(stderr, "DRM zero-copy unavailable, trying CPU copy path\n");
    }

    if (v4l2m2m_available() && v4l2m2m_init_codec(width, height, hw_codec) == 0) {
      backend = BACKEND_V4L2_COPY;
      printf("Using hardware video decoder: %s V4L2 M2M (%dx%d)\n",
             hw_codec_name, width, height);
      return 0;
    }
    /* Refuse to degrade silently. An A53 cannot software-decode 2K60 HEVC: the
     * FFmpeg path would crawl at a few fps while the process stays alive and
     * systemd keeps reporting the service as healthy, so the failure shows up
     * as a frozen picture that never recovers. Returning -1 makes moonlight
     * abort the session instead, and Restart=always picks it up.
     * MOONLIGHT_ALLOW_SW_DECODE=1 restores the old behaviour for debugging. */
    const char *allow_sw = getenv("MOONLIGHT_ALLOW_SW_DECODE");
    if (allow_sw == NULL || allow_sw[0] == '\0' || allow_sw[0] == '0') {
      fprintf(stderr,
              "%s hardware decoder unavailable - refusing to fall back to "
              "software decoding (set MOONLIGHT_ALLOW_SW_DECODE=1 to allow it)\n",
              hw_codec_name);
      /* The generic "return -1" is swallowed by libgamestream: the session goes
       * on with no video at all and nothing ever restarts it. Exit instead, so
       * systemd sees the failure and the autostart re-runs its host recovery.
       * Nothing to leak here - the hardware decoder is exactly what failed to
       * initialise. */
      exit(EXIT_FAILURE);
    }
    fprintf(stderr, "%s hardware decoder unavailable, falling back to FFmpeg\n",
            hw_codec_name);
  }

  if (ffmpeg_init(videoFormat, width, height, SLICE_THREADING, SDL_BUFFER_FRAMES, SLICES_PER_FRAME) < 0) {
    fprintf(stderr, "Couldn't initialize video decoding\n");
    return -1;
  }
  backend = BACKEND_FFMPEG;

  return 0;
}

static void sdl_cleanup() {
  switch (backend) {
  case BACKEND_DRM:
    drm_video_cleanup();
    break;
  case BACKEND_V4L2_COPY:
    v4l2m2m_cleanup();
    break;
  case BACKEND_FFMPEG:
    ffmpeg_destroy();
    break;
  default:
    break;
  }
  backend = BACKEND_NONE;
}

static void push_frame(unsigned char** planes, int* strides) {
  if (SDL_LockMutex(mutex) != 0) {
    fprintf(stderr, "Couldn't lock mutex\n");
    return;
  }

  sdlNextFrame++;

  SDL_Event event;
  event.type = SDL_USEREVENT;
  event.user.code = SDL_CODE_FRAME;
  event.user.data1 = planes;
  event.user.data2 = strides;
  SDL_PushEvent(&event);

  SDL_UnlockMutex(mutex);
}

static int sdl_submit_decode_unit(PDECODE_UNIT decodeUnit) {
  PLENTRY entry = decodeUnit->bufferList;
  int length = 0;

  ensure_buf_size(&decode_buffer, &decode_buffer_size, decodeUnit->fullLength + AV_INPUT_BUFFER_PADDING_SIZE);

  while (entry != NULL) {
    memcpy((unsigned char*)decode_buffer + length, entry->data, entry->length);
    length += entry->length;
    entry = entry->next;
  }

  if (backend == BACKEND_DRM) {
    if (drm_video_submit((unsigned char*)decode_buffer, length) < 0)
      return DR_NEED_IDR;
    return DR_OK;
  }

  if (backend == BACKEND_V4L2_COPY) {
    if (v4l2m2m_feed((unsigned char*)decode_buffer, length) < 0)
      return DR_NEED_IDR;

    // Wait briefly for the first frame, then take whatever else is already done
    int produced = 0;
    while (v4l2m2m_dequeue(produced == 0 ? HW_DECODE_WAIT_MS : 0) > 0) {
      push_frame(v4l2m2m_planes(), v4l2m2m_strides());
      produced++;
    }

    return DR_OK;
  }

  ffmpeg_decode((unsigned char*)decode_buffer, length);

  AVFrame* frame = ffmpeg_get_frame(false);
  if (frame != NULL)
    push_frame(frame->data, frame->linesize);

  return DR_OK;
}

DECODER_RENDERER_CALLBACKS decoder_callbacks_sdl = {
  .setup = sdl_setup,
  .cleanup = sdl_cleanup,
  .submitDecodeUnit = sdl_submit_decode_unit,
  // The hardware paths wait on the VPU inside submitDecodeUnit, so they can't
  // be declared as direct submit, and whole frames keep the pipeline simple.
  .capabilities = CAPABILITY_REFERENCE_FRAME_INVALIDATION_HEVC,
};
