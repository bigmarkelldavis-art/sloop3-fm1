/* SPDX-License-Identifier: GPL-3.0-only
 * USB AUDIO IN (usbin.c): what a host plays to the FM-1's USB audio output, 16-bit stereo at 44.1 or 48 kHz, brought to the
 * engine's rate as one mono channel ((L + R) / 2: the RAM is tight). The USB driver (hal/fm1/usb.c, from the 2 kHz tick)
 * pushes each packet it receives and it is converted at once, in integers (the tick must not touch the float registers), by
 * linear interpolation into a ring at the engine's rate; the main loop pulls one block of the engine's frames a block.
 * The two clocks differ a little (the host's, the FM-1's 44117.6 Hz): the conversion step follows how full the ring is.
 * The engine renders four blocks at once for each half of its audio buffer, so the ring takes a burst of 256 frames:
 * UIN_N is more than that, a packet's worth and the main loop being a little late.
 * Written for this project; no hardware here (it runs on the PC twin and in tests/usbin_test.c). */
#pragma once
#include <stdint.h>

#ifndef UIN_N
#define UIN_N 448u                          /* the ring: frames of 16 bits (896 bytes; 10 ms) */
#endif
#ifndef UIN_TARGET
#define UIN_TARGET 212u                     /* the average fill the conversion steers to */
#endif
#define UIN_ENGINE_HZ 44117.647f            /* the FM-1's audio rate (hal/fm1/audio.c) */

void uin_reset(void);                       /* a stream starts or stops: the ring emptied, the lock lost */
void uin_set_rate(uint32_t hz);             /* the host's rate: 44100 or 48000 */
/* one received packet's frames (L | R << 16, 16 bits each): the producer, the USB tick (integers only) */
void uin_push(const uint32_t *frames, uint32_t n);
/* n frames into m (-1..1): the consumer, the main loop. Silence while there is no stream, and for a moment after it
 * begins (it waits for the ring to fill). Returns 1 if the frames are the host's. */
int uin_pull(float *m, uint32_t n);
void uin_stream(int on);                    /* the host's alternate setting: 1 streaming, 0 not */
int uin_active(void);                       /* streaming and locked (frames are coming through) */
uint32_t uin_fill(void);                    /* frames in the ring */
uint32_t uin_lost(void);                    /* frames dropped (a full ring) + frames missing (an empty one), since reset */
