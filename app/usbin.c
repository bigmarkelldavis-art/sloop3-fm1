/* SPDX-License-Identifier: GPL-3.0-only
 * USB AUDIO IN (usbin.h): a single-producer single-consumer ring of 16-bit stereo frames, read at the engine's rate by
 * linear interpolation, the read step steered by how full the ring is. Written for this project. */
#include "usbin.h"

static uint32_t ring[UIN_N];
static volatile uint32_t w;                 /* written by the producer only */
static volatile uint32_t rd;                /* written by the consumer only */
static volatile uint8_t streaming, flush;   /* the producer sets them; the consumer acts on them */
static volatile uint32_t host_hz = 44100u;
static volatile uint32_t lost;              /* (the producer's drops and the consumer's holds: a count for the screen) */
/* the consumer's */
static uint8_t locked, dry_blocks;
static float frac, avg, a_l, a_r, b_l, b_r;

void uin_set_rate(uint32_t hz) { host_hz = hz >= 46050u ? 48000u : 44100u; flush = 1; }
void uin_reset(void) { flush = 1; lost = 0; }
void uin_stream(int on) { streaming = (uint8_t)(on != 0); flush = 1; }
int uin_active(void) { return streaming && locked; }
uint32_t uin_fill(void) { return w - rd; }
uint32_t uin_lost(void) { return lost; }

void uin_push(const uint32_t *f, uint32_t n)
{
    if (!streaming) return;
    uint32_t wi = w;
    for (uint32_t i = 0; i < n; i++) {
        if (wi - rd >= UIN_N) { lost += n - i; break; }     /* full: the rest of the packet is dropped */
        ring[wi & (UIN_N - 1u)] = f[i];
        wi++;
    }
    w = wi;
}

static float fl(uint32_t fr) { return (float)(int16_t)(fr & 0xFFFFu) * (1.0f / 32768.0f); }
static float fr_(uint32_t fr) { return (float)(int16_t)(fr >> 16) * (1.0f / 32768.0f); }

int uin_pull(float *l, float *r, uint32_t n)
{
    if (flush) { flush = 0; rd = w; locked = 0; frac = 0.0f; dry_blocks = 0; }
    if (!streaming) { for (uint32_t i = 0; i < n; i++) l[i] = r[i] = 0.0f; return 0; }
    uint32_t fill = w - rd;
    if (!locked) {
        if (fill < UIN_TARGET) { for (uint32_t i = 0; i < n; i++) l[i] = r[i] = 0.0f; return 0; }
        uint32_t f0 = ring[rd & (UIN_N - 1u)];
        b_l = fl(f0); b_r = fr_(f0); a_l = b_l; a_r = b_r;           /* (starts from the first frame, no jump) */
        rd++; frac = 0.0f; avg = (float)fill; locked = 1; dry_blocks = 0;
    }
    avg += ((float)fill - avg) * 0.05f;
    float corr = (avg - (float)UIN_TARGET) * (1.0f / (float)UIN_TARGET) * 0.002f;
    corr = corr > 0.004f ? 0.004f : corr < -0.004f ? -0.004f : corr;
    float step = (float)host_hz * (1.0f / UIN_ENGINE_HZ) * (1.0f + corr);
    uint32_t held = 0;
    for (uint32_t i = 0; i < n; i++) {
        l[i] = a_l + (b_l - a_l) * frac;
        r[i] = a_r + (b_r - a_r) * frac;
        frac += step;
        while (frac >= 1.0f) {
            frac -= 1.0f; a_l = b_l; a_r = b_r;
            if (rd != w) { uint32_t f = ring[rd & (UIN_N - 1u)]; rd++; b_l = fl(f); b_r = fr_(f); }
            else held++;                                            /* (the ring empty: the last frame again) */
        }
    }
    if (held) { lost += held; if (++dry_blocks >= 16u) { locked = 0; dry_blocks = 0; } }   /* (~23 ms dry: the host has stopped) */
    else dry_blocks = 0;
    return 1;
}
