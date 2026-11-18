/* SPDX-License-Identifier: GPL-3.0-only
 * USB AUDIO IN (usbin.c): what a host plays to the FM-1's USB audio output endpoint, 16-bit stereo at 44.1 or 48 kHz,
 * brought to the engine's rate. The USB driver (hal/fm1/usb.c, from the 2 kHz tick) pushes the packets it receives;
 * the main loop pulls one block of the engine's frames a block. The two clocks differ a little (the host's, the
 * FM-1's 44117.6 Hz): the read step follows how full the ring is, so it neither empties nor overflows.
 * Written for this project; no hardware here (it runs on the PC twin and in tests/usbin_test.c). */
#pragma once
#include <stdint.h>

#define UIN_N 512u                          /* the ring: frames, a power of two (11.6 ms at 44.1 kHz) */
#define UIN_TARGET 160u                     /* the fill the read step steers to (3.6 ms) */
#define UIN_ENGINE_HZ 44117.647f            /* the FM-1's audio rate (hal/fm1/audio.c) */

void uin_reset(void);                       /* a stream starts or stops: the ring emptied, the lock lost */
void uin_set_rate(uint32_t hz);             /* the host's rate: 44100 or 48000 */
/* one received packet's frames (L | R << 16, 16 bits each): the producer, the USB tick */
void uin_push(const uint32_t *frames, uint32_t n);
/* n frames at the engine's rate into l and r (-1..1): the consumer, the main loop. Silence while there is no stream,
 * and for a moment after it begins (it waits for UIN_TARGET frames). Returns 1 if the frames are the host's. */
int uin_pull(float *l, float *r, uint32_t n);
void uin_stream(int on);                    /* the host's alternate setting: 1 streaming, 0 not */
int uin_active(void);                       /* streaming and locked (frames are coming through) */
uint32_t uin_fill(void);                    /* frames in the ring */
uint32_t uin_lost(void);                    /* frames dropped (a full ring) + frames missing (an empty one), since reset */
