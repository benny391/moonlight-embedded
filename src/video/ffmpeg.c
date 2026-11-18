/*
 * This file is part of Moonlight Embedded.
 *
 * Based on Moonlight Pc implementation
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

#include "ffmpeg.h"

#ifdef HAVE_VAAPI
#include "ffmpeg_vaapi.h"
#endif

#include <Limelight.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>

#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// General decoder and renderer state
static AVPacket* pkt;
static const AVCodec* decoder;
static AVCodecContext* decoder_ctx;
static AVFrame** dec_frames;

static int dec_frames_cnt;
static int current_frame, next_frame;

// Hardware (V4L2 M2M) support: decoder output is NV12, renderer expects YUV420P
static bool hw_decoder = false;
static struct SwsContext* sws_ctx = NULL;
static AVFrame** conv_frames = NULL;

enum decoders ffmpeg_decoder;

#define BYTES_PER_PIXEL 4

static bool force_software(void) {
  const char* v = getenv("MOONLIGHT_FORCE_SW");
  return v != NULL && v[0] != '\0' && strcmp(v, "0") != 0;
}

// Converts a decoder frame to the packed YUV420P layout the renderer expects.
static AVFrame* convert_to_yuv420p(AVFrame* src, int slot) {
  if (slot < 0 || slot >= dec_frames_cnt || conv_frames == NULL)
    return src;

  AVFrame* dst = conv_frames[slot];
  if (dst == NULL || dst->width != src->width || dst->height != src->height) {
    if (dst != NULL)
      av_frame_free(&dst);
    dst = av_frame_alloc();
    if (dst == NULL)
      return src;
    dst->format = AV_PIX_FMT_YUV420P;
    dst->width = src->width;
    dst->height = src->height;
    if (av_frame_get_buffer(dst, 32) < 0) {
      av_frame_free(&dst);
      return src;
    }
    conv_frames[slot] = dst;
  }

  if (sws_ctx == NULL) {
    sws_ctx = sws_getContext(src->width, src->height, src->format,
                             src->width, src->height, AV_PIX_FMT_YUV420P,
                             SWS_POINT, NULL, NULL, NULL);
    if (sws_ctx == NULL) {
      fprintf(stderr, "Couldn't create color conversion context\n");
      return src;
    }
  }

  sws_scale(sws_ctx, (const uint8_t* const*)src->data, src->linesize, 0,
            src->height, dst->data, dst->linesize);
  return dst;
}

// This function must be called before
// any other decoding functions
int ffmpeg_init(int videoFormat, int width, int height, int perf_lvl, int buffer_count, int thread_count) {
  // Initialize the avcodec library and register codecs
  av_log_set_level(AV_LOG_QUIET);
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58,10,100)
  avcodec_register_all();
#endif

  pkt = av_packet_alloc();
  if (pkt == NULL) {
    printf("Couldn't allocate packet\n");
    return -1;
  }

  ffmpeg_decoder = perf_lvl & VAAPI_ACCELERATION ? VAAPI : SOFTWARE;

  bool soft_only = force_software();

  for (int try = 0; try < 6; try++) {
    if (videoFormat & VIDEO_FORMAT_MASK_H264) {
      if (ffmpeg_decoder == SOFTWARE) {
        if (try == 0) decoder = avcodec_find_decoder_by_name("h264_nvv4l2"); // Tegra
        if (try == 1) decoder = avcodec_find_decoder_by_name("h264_nvmpi"); // Tegra
        if (try == 2) decoder = avcodec_find_decoder_by_name("h264_omx"); // VisionFive
        if (try == 3) decoder = soft_only ? NULL : NULL; // Stateful V4L2
      }
      if (try == 4) decoder = avcodec_find_decoder_by_name("h264"); // Software and hwaccel
    } else if (videoFormat & VIDEO_FORMAT_MASK_H265) {
      if (ffmpeg_decoder == SOFTWARE) {
        if (try == 0) decoder = avcodec_find_decoder_by_name("hevc_nvv4l2"); // Tegra
        if (try == 1) decoder = avcodec_find_decoder_by_name("hevc_nvmpi"); // Tegra
        if (try == 2) decoder = avcodec_find_decoder_by_name("hevc_omx"); // VisionFive
        if (try == 3) decoder = soft_only ? NULL : NULL; // Stateful V4L2
      }
      if (try == 4) decoder = avcodec_find_decoder_by_name("hevc"); // Software and hwaccel
    } else if (videoFormat & VIDEO_FORMAT_MASK_AV1) {
      if (ffmpeg_decoder == SOFTWARE) {
        if (try == 0) decoder = avcodec_find_decoder_by_name("libdav1d");
      }
      if (try == 1) decoder = avcodec_find_decoder_by_name("av1"); // Hwaccel
    } else {
      printf("Video format not supported\n");
      return -1;
    }

    // Skip this decoder if it isn't compiled into FFmpeg
    if (!decoder) {
      continue;
    }

    decoder_ctx = avcodec_alloc_context3(decoder);
    if (decoder_ctx == NULL) {
      printf("Couldn't allocate context\n");
      return -1;
    }

    // Use low delay decoding
    decoder_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;

    // Allow display of corrupt frames and frames missing references
    decoder_ctx->flags |= AV_CODEC_FLAG_OUTPUT_CORRUPT;
    decoder_ctx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;

    // Hardware decoders reject truncated/odd streams if we explode on errors
    if (strstr(decoder->name, "v4l2m2m") == NULL)
      decoder_ctx->err_recognition = AV_EF_EXPLODE;

    if (perf_lvl & SLICE_THREADING) {
      decoder_ctx->thread_type = FF_THREAD_SLICE;
      decoder_ctx->thread_count = thread_count;
    } else {
      decoder_ctx->thread_count = 1;
    }

    decoder_ctx->width = width;
    decoder_ctx->height = height;
    decoder_ctx->pix_fmt = AV_PIX_FMT_YUV420P;

    int err = avcodec_open2(decoder_ctx, decoder, NULL);
    if (err < 0) {
      printf("Couldn't open codec: %s\n", decoder->name);
      avcodec_free_context(&decoder_ctx);
      decoder_ctx = NULL;
      continue;
    }

    // Success: stop trying lower-priority decoders
    hw_decoder = strstr(decoder->name, "v4l2m2m") != NULL;
    break;
  }

  if (decoder_ctx == NULL || decoder == NULL) {
    printf("Couldn't find decoder\n");
    return -1;
  }

  printf("Using FFmpeg decoder: %s%s\n", decoder->name,
         hw_decoder ? " (hardware)" : " (software)");

  dec_frames_cnt = buffer_count;
  dec_frames = calloc(buffer_count, sizeof(AVFrame*));
  if (dec_frames == NULL) {
    fprintf(stderr, "Couldn't allocate frames");
    return -1;
  }

  if (hw_decoder) {
    conv_frames = calloc(buffer_count, sizeof(AVFrame*));
    if (conv_frames == NULL) {
      fprintf(stderr, "Couldn't allocate conversion frames");
      return -1;
    }
  }

  for (int i = 0; i < buffer_count; i++) {
    dec_frames[i] = av_frame_alloc();
    if (dec_frames[i] == NULL) {
      fprintf(stderr, "Couldn't allocate frame");
      return -1;
    }
  }

  #ifdef HAVE_VAAPI
  if (ffmpeg_decoder == VAAPI)
    vaapi_init(decoder_ctx);

  #endif

  return 0;
}

// This function must be called after
// decoding is finished
void ffmpeg_destroy(void) {
  av_packet_free(&pkt);
  if (decoder_ctx) {
    avcodec_free_context(&decoder_ctx);
  }
  if (dec_frames) {
    for (int i = 0; i < dec_frames_cnt; i++) {
      if (dec_frames[i])
        av_frame_free(&dec_frames[i]);
    }
    free(dec_frames);
    dec_frames = NULL;
  }
  if (conv_frames) {
    for (int i = 0; i < dec_frames_cnt; i++) {
      if (conv_frames[i])
        av_frame_free(&conv_frames[i]);
    }
    free(conv_frames);
    conv_frames = NULL;
  }
  if (sws_ctx) {
    sws_freeContext(sws_ctx);
    sws_ctx = NULL;
  }
  hw_decoder = false;
}

AVFrame* ffmpeg_get_frame(bool native_frame) {
  int err = avcodec_receive_frame(decoder_ctx, dec_frames[next_frame]);
  if (err == 0) {
    current_frame = next_frame;
    next_frame = (current_frame+1) % dec_frames_cnt;

    if (ffmpeg_decoder == SOFTWARE || native_frame) {
      AVFrame* frame = dec_frames[current_frame];
      // Hardware decoders (V4L2 M2M) deliver NV12 while the renderer wants
      // planar YUV420P, so convert before handing the frame over.
      if (hw_decoder && !native_frame && frame->format != AV_PIX_FMT_YUV420P)
        return convert_to_yuv420p(frame, current_frame);
      return frame;
    }
  } else if (err != AVERROR(EAGAIN)) {
    char errorstring[512];
    av_strerror(err, errorstring, sizeof(errorstring));
    fprintf(stderr, "Receive failed - %d/%s\n", err, errorstring);
  }
  return NULL;
}

// packets must be decoded in order
// indata must be inlen + AV_INPUT_BUFFER_PADDING_SIZE in length
int ffmpeg_decode(unsigned char* indata, int inlen) {
  int err;

  pkt->data = indata;
  pkt->size = inlen;

  err = avcodec_send_packet(decoder_ctx, pkt);
  if (err < 0) {
    char errorstring[512];
    av_strerror(err, errorstring, sizeof(errorstring));
    fprintf(stderr, "Decode failed - %s\n", errorstring);
  }

  return err < 0 ? err : 0;
}
