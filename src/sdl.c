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

#ifdef HAVE_SDL

#include "sdl.h"
#include "input/sdl.h"
#include "video/drmzero.h"

#include <Limelight.h>

static bool done;
static int fullscreen_flags;
static bool drm_output;

static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_Texture *bmp;

SDL_mutex *mutex;

int sdlCurrentFrame, sdlNextFrame;

void sdl_init(int width, int height, bool fullscreen) {
  sdlCurrentFrame = sdlNextFrame = 0;

  // When the video is scanned out through a DRM plane, SDL only has to deliver
  // input events; giving it a window would fight over the CRTC.
  drm_output = drm_video_available() != 0;
  if (drm_output) {
    printf("SDL: video goes through DRM, SDL used for input only\n");
    setenv("SDL_VIDEODRIVER", "dummy", 1);
  }

  if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS)) {
    fprintf(stderr, "Could not initialize SDL - %s\n", SDL_GetError());
    exit(1);
  }

  fullscreen_flags = fullscreen?SDL_WINDOW_FULLSCREEN:0;
  /* The dummy video driver has no GL support, and the window is only kept for
   * the input code (mouse position, focus); asking for an OpenGL window there
   * fails outright. The real video output is the DRM plane either way. */
  Uint32 window_flags = fullscreen_flags;
  if (!drm_output)
    window_flags |= SDL_WINDOW_OPENGL;
  window = SDL_CreateWindow("Moonlight", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, window_flags);
  if(!window) {
    fprintf(stderr, "SDL: could not create window - %s\n", SDL_GetError());
    exit(1);
  }

  mutex = SDL_CreateMutex();
  if (!mutex) {
    fprintf(stderr, "Couldn't create mutex\n");
    exit(1);
  }

  if (drm_output)
    return;

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) {
    printf("SDL_CreateRenderer failed: %s\n", SDL_GetError());
    exit(1);
  }

  bmp = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_YV12, SDL_TEXTUREACCESS_STREAMING, width, height);
  if (!bmp) {
    fprintf(stderr, "SDL: could not create texture - exiting\n");
    exit(1);
  }
}

void sdl_loop() {
  SDL_Event event;

  if (!drm_output)
    SDL_SetRelativeMouseMode(SDL_TRUE);

  while(!done && SDL_WaitEvent(&event)) {
    switch (sdlinput_handle_event(window, &event)) {
    case SDL_QUIT_APPLICATION:
      done = true;
      break;
    case SDL_TOGGLE_FULLSCREEN:
      fullscreen_flags ^= SDL_WINDOW_FULLSCREEN;
      if (!drm_output)
        SDL_SetWindowFullscreen(window, fullscreen_flags);
      break;
    case SDL_MOUSE_GRAB:
      if (!drm_output) {
        SDL_ShowCursor(SDL_ENABLE);
        SDL_SetRelativeMouseMode(SDL_TRUE);
      }
      break;
    case SDL_MOUSE_UNGRAB:
      if (!drm_output) {
        SDL_SetRelativeMouseMode(SDL_FALSE);
        SDL_ShowCursor(SDL_DISABLE);
      }
      break;
    default:
      if (event.type == SDL_QUIT)
        done = true;
      else if (event.type == SDL_USEREVENT) {
        if (event.user.code == SDL_CODE_FRAME) {
          if (renderer == NULL) {
            // DRM path: the decoder already handed the frame to the display
          } else if (++sdlCurrentFrame <= sdlNextFrame - SDL_BUFFER_FRAMES) {
            //Skip frame
          } else if (SDL_LockMutex(mutex) == 0) {
            Uint8** data = ((Uint8**) event.user.data1);
            int* linesize = ((int*) event.user.data2);
            SDL_UpdateYUVTexture(bmp, NULL, data[0], linesize[0], data[1], linesize[1], data[2], linesize[2]);
            SDL_UnlockMutex(mutex);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, bmp, NULL, NULL);
            SDL_RenderPresent(renderer);
          } else
            fprintf(stderr, "Couldn't lock mutex\n");
        }
      }
    }
  }

  SDL_DestroyWindow(window);
  SDL_Quit();
}

#endif /* HAVE_SDL */
