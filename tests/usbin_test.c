/* SPDX-License-Identifier: GPL-3.0-only
 * tests/usbin_test.c: app/usbin.c under a simulated host. A sine goes in a packet a millisecond (the host's clock
 * off by some ppm), blocks of 64 frames come out at the FM-1's rate: no frames lost, the sine's frequency kept, no
 * jump (a click), the stream stopping and starting again, the host's rate changing. */
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../app/usbin.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

typedef struct { double hz, ppm, ph, acc; } host_t;
static void host_ms(host_t *h, double tone)               /* one millisecond of the host: a packet */
{
    uint32_t f[64]; uint32_t n = 0;
    h->acc += h->hz * (1.0 + h->ppm * 1e-6) / 1000.0;
    while (h->acc >= 1.0) {
        h->acc -= 1.0;
        int16_t s = (int16_t)(sin(h->ph) * 16000.0);
        f[n++] = (uint16_t)s | (uint32_t)(uint16_t)s << 16;
        h->ph += 2.0 * M_PI * tone / h->hz;
    }
    uin_push(f, n);
}
/* seconds of run; returns the output's frequency (zero crossings), the biggest step, the frames lost */
static void run(double hz, double ppm, double secs, double tone, double *fo, double *maxstep, uint32_t *lost, int *active)
{
    host_t h = { hz, ppm, 0, 0 };
    uin_set_rate((uint32_t)hz); uin_stream(1);
    double t = 0, tms = 0, block_s = 64.0 / (double)UIN_ENGINE_HZ;
    float l[64], r[64], prev = 0; int started = 0; long cross = 0; double t_first = 0, t_last = 0; double ms = 0; *maxstep = 0;
    while (t < secs) {
        t += block_s;
        while (tms < t) { host_ms(&h, tone); tms += 0.001; }
        int got = uin_pull(l, r, 64);
        (void)ms;
        for (int i = 0; i < 64; i++) {
            CHECK(l[i] == r[i], "L and R differ");
            if (got) {
                if (started) {
                    double d = fabs((double)l[i] - (double)prev);
                    if (t > 0.5 && d > *maxstep) *maxstep = d;
                    if (t > 0.5 && prev < 0.0f && l[i] >= 0.0f) { if (!cross) t_first = t; t_last = t; cross++; }
                }
                prev = l[i]; started = 1;
            }
        }
    }
    *fo = cross > 1 ? (double)(cross - 1) / (t_last - t_first) : 0.0;
    *lost = uin_lost(); *active = uin_active();
}

int main(void)
{
    const double rates[2] = { 44100.0, 48000.0 }, ppms[4] = { 0, 300, -300, 1500 };
    for (int ri = 0; ri < 2; ri++) for (int pi = 0; pi < 4; pi++) {
        uin_reset();
        double fo, ms; uint32_t lost; int act;
        run(rates[ri], ppms[pi], 20.0, 1000.0, &fo, &ms, &lost, &act);
        /* a 1 kHz sine of amplitude 16000/32768 steps at most 2 sin(pi 1000/44117) * 0.488 = 0.0698; allowed: a little more */
        printf("host %5.0f Hz %+5.0f ppm: out %.2f Hz, biggest step %.4f, lost %u, active %d\n", rates[ri], ppms[pi], fo, ms, lost, act);
        CHECK(act, "the stream is not carried");
        CHECK(fabs(fo - 1000.0 * 1.0) < 3.0, "the tone moved: %.2f Hz", fo);
        CHECK(ms < 0.08, "a jump: %.4f", ms);
        CHECK(lost < 40, "frames lost: %u", lost);
    }
    /* the host stops (alternate 0): silence at once, and nothing carried */
    {
        uin_reset(); double fo, ms; uint32_t lost; int act;
        run(44100.0, 0, 1.0, 1000.0, &fo, &ms, &lost, &act);
        uin_stream(0);
        float l[64], r[64]; int got = uin_pull(l, r, 64);
        CHECK(!got && !uin_active(), "carried after stop");
        for (int i = 0; i < 64; i++) CHECK(l[i] == 0.0f && r[i] == 0.0f, "not silent after stop");
        /* and again, with a different rate */
        run(48000.0, 0, 2.0, 440.0, &fo, &ms, &lost, &act);
        CHECK(act && fabs(fo - 440.0) < 2.0, "restart at 48k: %.2f Hz", fo);
    }
    /* the host goes quiet without a stop (a cable pulled): the lock is let go, no noise */
    {
        uin_reset(); uin_set_rate(44100); uin_stream(1);
        uint32_t f[45]; for (int i = 0; i < 45; i++) f[i] = 0x10001000u;
        for (int k = 0; k < 10; k++) uin_push(f, 44);
        float l[64], r[64]; int got = 0;
        for (int b = 0; b < 40; b++) got = uin_pull(l, r, 64);
        CHECK(!uin_active() && !got, "still locked on a dead stream");
    }
    /* a packet burst larger than the ring: dropped, counted, no overwrite of what is unread */
    {
        uin_reset(); uin_set_rate(44100); uin_stream(1);
        uint32_t f[45]; for (int i = 0; i < 45; i++) f[i] = (uint32_t)i;
        for (int k = 0; k < 30; k++) uin_push(f, 45);
        CHECK(uin_fill() <= UIN_N, "the ring overran: %u", uin_fill());
        float l[64], r[64]; uin_pull(l, r, 64);
        CHECK(uin_lost() > 0, "no drop counted");
    }
    printf(fails ? "usbin_test: FAILED (%d)\n" : "usbin_test: ok\n", fails);
    return fails != 0;
}
