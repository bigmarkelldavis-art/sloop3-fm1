/* SPDX-License-Identifier: GPL-3.0-only
 * tests/usbrec_test.c: app/usbrec.c recording a sine as the firmware would (sectors handed out as they fill, written to
 * a simulated flash, the header last), then the slot read back by the firmware's own reader (engine/sample.c:
 * smp_user_head_ok, smp_user_scan, smp_zone_of) and decoded as the SAMPLE engine decodes it. */
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../app/usbrec.h"
#include "../engine/sample.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static uint8_t flash[SMP_USR_SIZE], buf[UREC_BUF];
static int sectors_seen[SMP_USR_SIZE / 4096];

/* the flash's side: every sector handed out is written; `lag` blocks pass before it counts as written */
static void pump(usbrec *r, int *lag_q, int lag, uint32_t *queue, int *qn)
{
    uint32_t sec, from, n; const uint8_t *d;
    while ((d = urec_take(r, &sec, &from, &n)) != 0) {
        CHECK(sec < SMP_USR_SIZE / 4096 && from + n <= 4096, "bad sector %u %u %u", sec, from, n);
        if (sec >= SMP_USR_SIZE / 4096) return;
        memcpy(flash + sec * 4096 + from, d, n);
        sectors_seen[sec]++;
        queue[(*qn)++] = sec; lag_q[sec & 1] = lag;
    }
    for (int i = 0; i < *qn; ) {                            /* a sector written once its lag has passed */
        if (lag_q[queue[i] & 1]-- <= 0) { urec_written(r, queue[i]); memmove(queue + i, queue + i + 1, (size_t)(*qn - i - 1) * 4u); (*qn)--; } else i++;
    }
}

static int decode(const uint8_t *slot, int16_t *out, uint32_t n)       /* the SAMPLE engine's decoder (sample.c next()) */
{
    const uint8_t *d = slot + 512; int pred = 0, idx = 0;
    for (uint32_t pos = 0; pos < n; pos++) {
        uint32_t b = d[pos >> 1], code = (pos & 1u) ? b >> 4 : b & 15u;
        int step = smp_ima_step[idx], vd = step >> 3;
        if (code & 4u) vd += step;
        if (code & 2u) vd += step >> 1;
        if (code & 1u) vd += step >> 2;
        pred += (code & 8u) ? -vd : vd; pred = pred < -32768 ? -32768 : pred > 32767 ? 32767 : pred;
        idx += smp_ima_idx[code & 7u]; idx = idx < 0 ? 0 : idx > 88 ? 88 : idx;
        out[pos] = (int16_t)pred;
    }
    return 0;
}

static void take(int div, double secs, double tone, int lead_quiet_blocks, int lag, uint32_t *samples, double *snr_db, int expect_full)
{
    usbrec r; uint32_t queue[8]; int qn = 0, lag_q[2] = { 0, 0 };
    memset(flash, 0xFF, sizeof flash); memset(sectors_seen, 0, sizeof sectors_seen);
    urec_begin(&r, buf, SMP_USR_SIZE, div);
    float l[64], rr[64]; double ph = 0; long blocks = (long)(secs * 44117.647 / 64.0);
    double *ref = malloc(sizeof(double) * (size_t)(blocks * 64 + 64)); long nref = 0;
    for (long b = 0; b < blocks && r.on; b++) {
        for (int i = 0; i < 64; i++) {
            float v = b < lead_quiet_blocks ? 0.0005f * (float)sin(ph) : (float)(0.5 * sin(ph));
            ph += 2.0 * M_PI * tone / 44117.647; l[i] = rr[i] = v;
        }
        int was = r.started;
        urec_feed(&r, l, rr, 64);
        if (r.started) {                                    /* the reference: what was fed from the first loud block, decimated */
            int from = was ? 0 : 0;
            double acc = 0; int c = 0; static double carry; static int cc;
            if (!was) { carry = 0; cc = 0; }
            (void)from; (void)acc; (void)c;
            for (int i = 0; i < 64; i++) { carry += l[i]; if (++cc == div) { ref[nref++] = carry / div; carry = 0; cc = 0; } }
        }
        pump(&r, lag_q, lag, queue, &qn);
    }
    CHECK(r.err == 0, "an overrun with lag %d", lag);
    *samples = urec_stop(&r);
    for (int k = 0; k < 4; k++) pump(&r, lag_q, 0, queue, &qn), pump(&r, lag_q, 0, queue, &qn);
    CHECK(r.full == (uint8_t)expect_full, "full = %d, wanted %d", r.full, expect_full);
    uint8_t h[UREC_HEAD]; urec_header(&r, h, "USB REC"); memcpy(flash, h, UREC_HEAD);
    /* every sector that holds data was handed out once, in full */
    uint32_t last = (512u + r.bytes + 4095u) / 4096u;
    for (uint32_t s = 0; s < last; s++) CHECK(sectors_seen[s] == 1, "sector %u written %d times", s, sectors_seen[s]);
    /* the firmware's reader accepts it */
    CHECK(smp_user_head_ok(flash) == 1, "the header is refused");
    smp_user_scan(0, flash);
    CHECK(smp_user_nz(0) == 1, "smp_user_scan: %d zones", smp_user_nz(0));
    const smp_zone *z = smp_zone_of(SMP_USR | 0u);
    CHECK(z->n == *samples && z->rate == 65536u / (uint32_t)div && z->root16 == 960 && z->le == z->n - 1u && z->off == 0, "the zone");
    char nm[10]; smp_user_name(0, nm); CHECK(!strcmp(nm, "USB REC"), "name %s", nm);
    /* decoded, it is the signal */
    int16_t *dec = malloc(sizeof(int16_t) * (*samples + 1));
    decode(flash, dec, *samples);
    double es = 0, ee = 0; long m = (long)*samples < nref ? (long)*samples : nref;
    for (long i = 0; i < m; i++) { double s = ref[i] * 32767.0, e = (double)dec[i] - s; es += s * s; ee += e * e; }
    *snr_db = 10.0 * log10(es / (ee > 0 ? ee : 1));
    CHECK((long)*samples >= nref - 1 && (long)*samples <= nref + 1, "samples %u, the signal %ld", *samples, nref);
    free(ref); free(dec);
}

int main(void)
{
    static const int divs[3] = { 1, 2, 4 };
    for (int di = 0; di < 3; di++) {
        uint32_t n; double snr;
        take(divs[di], 1.5, 440.0, 0, 0, &n, &snr, 0);
        printf("div %d: 1.5 s -> %u samples, SNR %.1f dB\n", divs[di], n, snr);
        CHECK(snr > 25.0, "SNR %.1f dB at div %d", snr, divs[di]);
        take(divs[di], 1.5, 440.0, 0, 3, &n, &snr, 0);           /* a flash that is slow: three blocks to write a sector */
        CHECK(snr > 25.0, "SNR %.1f dB (lag) at div %d", snr, divs[di]);
    }
    { uint32_t n; double snr; take(1, 1.0, 1000.0, 40, 0, &n, &snr, 0);   /* 40 quiet blocks (58 ms) ignored */
      printf("armed: %u samples for 1.0 s - 58 ms\n", n); CHECK(n > 40000u && n < 44117u - 2500u + 600u, "armed: %u", n); }
    { uint32_t n; double snr; take(1, 6.0, 330.0, 0, 0, &n, &snr, 1);     /* more than fits: the slot fills and the take stops */
      printf("full: %u samples (room %u ms)\n", n, 0u); CHECK(n == 2u * (SMP_USR_SIZE - 512u), "full: %u", n); CHECK(snr > 25.0, "SNR when full %.1f", snr); }
    { usbrec r; urec_begin(&r, buf, SMP_USR_SIZE, 1);               /* a flash that never writes: the third sector finds its half busy */
      float l[64]; for (int i = 0; i < 64; i++) l[i] = 0.5f * (float)sin(i * 0.3);
      uint32_t sec, from, n; int got = 0;
      for (int b = 0; b < 400 && r.on; b++) { urec_feed(&r, l, l, 64); while (urec_take(&r, &sec, &from, &n)) got++; }
      CHECK(r.err == UREC_ERR_OVERRUN && got == 2, "overrun: err %d, sectors %d", r.err, got); }
    { usbrec r; urec_begin(&r, buf, SMP_USR_SIZE, 1);               /* nothing loud: nothing kept */
      float l[64] = { 0 }; for (int b = 0; b < 50; b++) urec_feed(&r, l, l, 64);
      CHECK(urec_stop(&r) == 0, "something recorded from silence"); }
    printf(fails ? "usbrec_test: FAILED (%d)\n" : "usbrec_test: ok\n", fails);
    return fails != 0;
}
