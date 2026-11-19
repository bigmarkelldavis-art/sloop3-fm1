/* SPDX-License-Identifier: GPL-3.0-only
 * USB AUDIO IN (usbin.h): the packets converted as they arrive (integer linear interpolation, the step steered by the ring's
 * fill) into a single-producer single-consumer ring of mono 16-bit samples at the engine's rate. Written for this project. */
#include "usbin.h"

#define BASE_44 65510u                       /* host Hz / the engine's Hz, 16.16: 44100 / 44117.647 */
#define BASE_48 71303u                       /* 48000 / 44117.647 */

static int16_t ring[UIN_N];
static volatile uint16_t w, rd;             /* the next to write (the producer's), the next to read (the consumer's) */
static volatile uint8_t streaming, flush;
static volatile uint32_t base = BASE_44;
static volatile uint32_t lost;              /* (a count for the screen; both sides add to it) */
/* the producer's: the conversion */
static int32_t prev;                        /* the last input sample */
static uint32_t frac, avg;                  /* the next output's place after prev (16.16); the fill, filtered (x 4096) */
static uint8_t started;
/* the consumer's */
static uint8_t locked, dry_blocks;
static float last;

uint32_t uin_fill(void) { int32_t f = (int32_t)w - (int32_t)rd; return (uint32_t)(f < 0 ? f + (int32_t)UIN_N : f); }
void uin_set_rate(uint32_t hz) { base = hz >= 46050u ? BASE_48 : BASE_44; }
void uin_reset(void) { started = 0; flush = 1; lost = 0; }
void uin_stream(int on) { streaming = (uint8_t)(on != 0); started = 0; flush = 1; }
int uin_active(void) { return streaming && locked; }
uint32_t uin_lost(void) { return lost; }

void uin_push(const uint32_t *f, uint32_t n)
{
    if (!streaming || !n) return;
    if (!started) { started = 1; prev = ((int32_t)(int16_t)(f[0] & 0xFFFFu) + (int32_t)(int16_t)(f[0] >> 16)) >> 1; frac = 0; avg = UIN_TARGET << 12; }
    /* the step: the base ratio, nudged by how far the average fill is from the target (a percent at most: it settles in a moment after a start) */
    int32_t e = ((int32_t)(avg >> 12) - (int32_t)UIN_TARGET), adj = e * 3;
    adj = adj > 655 ? 655 : adj < -655 ? -655 : adj;
    uint32_t step = (uint32_t)((int32_t)base + adj), wi = w;
    for (uint32_t i = 0; i < n; i++) {
        int32_t x = ((int32_t)(int16_t)(f[i] & 0xFFFFu) + (int32_t)(int16_t)(f[i] >> 16)) >> 1;
        while (frac < 65536u) {                                  /* the outputs between prev and x */
            int32_t o = prev + (int32_t)(((x - prev) * (int32_t)(frac >> 1)) >> 15);
            uint32_t nx = wi + 1u == UIN_N ? 0u : wi + 1u;
            if (nx == rd) lost++; else { ring[wi] = (int16_t)o; wi = nx; }       /* (full: this one is dropped) */
            frac += step;
        }
        frac -= 65536u; prev = x;
    }
    w = (uint16_t)wi;
    uint32_t fill = uin_fill();
    avg += (int32_t)((fill << 12) - avg) >> 8;                   /* (about a quarter of a second) */
}

int uin_pull(float *m, uint32_t n)
{
    if (flush) { flush = 0; rd = w; locked = 0; dry_blocks = 0; last = 0.0f; }
    if (!streaming) { for (uint32_t i = 0; i < n; i++) m[i] = 0.0f; return 0; }
    uint32_t fill = uin_fill();
    if (!locked) {
        if (fill < 200u) { for (uint32_t i = 0; i < n; i++) m[i] = 0.0f; return 0; }   /* (waits for some fill; the step then settles the rest) */
        locked = 1; dry_blocks = 0;
    }
    uint32_t r = rd, have = fill < n ? fill : n;
    for (uint32_t i = 0; i < have; i++) { last = (float)ring[r] * (1.0f / 32768.0f); m[i] = last; r = r + 1u == UIN_N ? 0u : r + 1u; }
    for (uint32_t i = have; i < n; i++) m[i] = last;             /* (the ring empty: the last sample again) */
    rd = (uint16_t)r;
    if (have < n) { lost += n - have; if (++dry_blocks >= 16u) { locked = 0; dry_blocks = 0; } }   /* (long dry: the host has stopped) */
    else dry_blocks = 0;
    return 1;
}

uint32_t uin_drain(int16_t *d, uint32_t max, uint32_t keep)
{
    if (flush) { flush = 0; rd = w; locked = 0; dry_blocks = 0; last = 0.0f; }
    if (!streaming) return 0;
    uint32_t fill = uin_fill();
    if (!locked) { if (fill < 200u) return 0; locked = 1; dry_blocks = 0; }
    if (fill <= keep) return 0;
    uint32_t n = fill - keep, r = rd;
    if (n > max) n = max;
    for (uint32_t i = 0; i < n; i++) { d[i] = ring[r]; r = r + 1u == UIN_N ? 0u : r + 1u; }
    rd = (uint16_t)r;
    return n;
}
