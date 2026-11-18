/* SPDX-License-Identifier: GPL-3.0-only
 * USB REC (usbrec.h). The encoder is IMA ADPCM as engine/sample.c decodes it (low nibble first, the same step and
 * index tables, the predictor rebuilt exactly as the decoder rebuilds it, so what is heard is what was encoded). */
#include "usbrec.h"
#include "../engine/sample.h"

static uint32_t crc_upd(uint32_t crc, uint8_t b)            /* CRC-32 (IEEE, as store.c's crc32_ieee) one byte at a time */
{
    crc = ~crc ^ b;
    for (int i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    return ~crc;
}

void urec_begin(usbrec *r, uint8_t *buf, uint32_t slot_size, int div)
{
    uint8_t *p = (uint8_t *)r;
    for (uint32_t i = 0; i < sizeof *r; i++) p[i] = 0;
    r->buf = buf; r->size = slot_size; r->div = (uint8_t)(div == 2 ? 2 : div == 4 ? 4 : 1);
    for (uint32_t i = 0; i < UREC_BUF; i++) buf[i] = 0xFF;
    r->on = 1;
}

static int encode(usbrec *r, int s)                          /* one sample (16 bits) to a code of 4 bits */
{
    int diff = s - r->pred, code = 0;
    if (diff < 0) { code = 8; diff = -diff; }
    int step = smp_ima_step[r->idx];
    if (diff >= step) { code |= 4; diff -= step; }
    step >>= 1;
    if (diff >= step) { code |= 2; diff -= step; }
    step >>= 1;
    if (diff >= step) code |= 1;
    step = smp_ima_step[r->idx];                             /* (the decoder's reconstruction, exactly) */
    int vd = step >> 3;
    if (code & 4) vd += step;
    if (code & 2) vd += step >> 1;
    if (code & 1) vd += step >> 2;
    int pr = r->pred + ((code & 8) ? -vd : vd);
    r->pred = (int16_t)(pr < -32768 ? -32768 : pr > 32767 ? 32767 : pr);
    int ix = r->idx + smp_ima_idx[code & 7];
    r->idx = (uint8_t)(ix < 0 ? 0 : ix > 88 ? 88 : ix);
    return code;
}

static void put_byte(usbrec *r, uint8_t b)
{
    uint32_t abs = UREC_DATA_AT + r->bytes, sec = abs / UREC_SECTOR, at = abs % UREC_SECTOR, half = sec & 1u;
    if (at == 0 || r->bytes == 0) {                          /* a sector begins: its half must be free (written out) */
        if (r->busy[half]) { r->err = UREC_ERR_OVERRUN; r->on = 0; return; }
        r->busy[half] = 1;
        for (uint32_t i = 0; i < UREC_SECTOR; i++) r->buf[half * UREC_SECTOR + i] = 0xFF;
    }
    r->buf[half * UREC_SECTOR + at] = b;
    r->crc = crc_upd(r->crc, b);
    r->bytes++;
    uint32_t end = UREC_DATA_AT + r->bytes;
    if (end % UREC_SECTOR == 0) r->ready = end / UREC_SECTOR;                 /* a sector is full */
    if (end >= r->size) { r->full = 1; r->on = 0; if (end % UREC_SECTOR) r->ready = end / UREC_SECTOR + 1u; }
}

static void put_sample(usbrec *r, int s)
{
    int code = encode(r, s);
    r->samples++;
    if (!r->odd) { r->lo = (uint8_t)code; r->odd = 1; }
    else { r->odd = 0; put_byte(r, (uint8_t)(r->lo | code << 4)); }
}

void urec_feed(usbrec *r, const float *l, const float *rt, uint32_t n)
{
    if (!r->on) return;
    if (!r->started) {
        float pk = 0.0f;
        for (uint32_t i = 0; i < n; i++) { float a = l[i] < 0 ? -l[i] : l[i], b = rt[i] < 0 ? -rt[i] : rt[i]; if (a > pk) pk = a; if (b > pk) pk = b; }
        if (pk < UREC_THRESH) return;                        /* armed: waiting for the sound */
        r->started = 1;
    }
    for (uint32_t i = 0; i < n && r->on; i++) {
        r->acc += 0.5f * (l[i] + rt[i]);
        if (++r->cnt < r->div) continue;
        float m = r->acc * (1.0f / (float)r->div) * 32767.0f;
        r->acc = 0.0f; r->cnt = 0;
        put_sample(r, (int)(m > 32767.0f ? 32767.0f : m < -32768.0f ? -32768.0f : m));
    }
}

const uint8_t *urec_take(usbrec *r, uint32_t *sector, uint32_t *from, uint32_t *n)
{
    while (r->taken < r->ready) {
        uint32_t sec = r->taken++, f = sec ? 0u : UREC_DATA_AT, last = UREC_DATA_AT + r->bytes;
        uint32_t end = (sec + 1u) * UREC_SECTOR <= last ? UREC_SECTOR : last - sec * UREC_SECTOR;
        if (end <= f) continue;                                                  /* (nothing in it) */
        *sector = sec; *from = f; *n = end - f;
        return r->buf + (sec & 1u) * UREC_SECTOR + f;
    }
    return 0;
}

void urec_written(usbrec *r, uint32_t sector) { r->busy[sector & 1u] = 0; }

uint32_t urec_stop(usbrec *r)
{
    if (r->odd) { r->odd = 0; put_byte(r, r->lo); }          /* (the last sample alone: its byte's high nibble 0) */
    r->on = 0;
    uint32_t end = UREC_DATA_AT + r->bytes;
    if (r->bytes && (end % UREC_SECTOR) && r->ready < end / UREC_SECTOR + 1u) r->ready = end / UREC_SECTOR + 1u;
    return r->bytes ? r->samples : 0u;
}

static void w32(uint8_t *b, uint32_t v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24); }
void urec_header(const usbrec *r, uint8_t *h, const char *name)
{
    for (uint32_t i = 0; i < UREC_HEAD; i++) h[i] = 0;
    w32(h, 0x504D5346u); h[4] = 1; h[5] = 0; h[6] = 1; h[7] = 0;                 /* "FSMP", version 1, one zone */
    for (int i = 0; i < 8 && name && name[i]; i++) h[8 + i] = (uint8_t)name[i];
    w32(h + 16, r->bytes); w32(h + 20, r->crc);
    uint8_t *z = h + 32;                                                       /* the zone: all of it, from C4 up and down */
    w32(z, 0); w32(z + 4, r->samples); w32(z + 8, 0); w32(z + 12, r->samples - 1u); w32(z + 16, 65536u / r->div);
    z[20] = 960 & 0xFF; z[21] = 960 >> 8; z[22] = 0; z[23] = 0; z[24] = 0; z[25] = 0; z[26] = 127; z[27] = 0;
}
uint32_t urec_len_ms(const usbrec *r) { return (uint32_t)((float)r->samples * (float)r->div * (1000.0f / 44117.647f)); }
uint32_t urec_room_ms(const usbrec *r) { return (uint32_t)((float)(2u * (r->size - UREC_DATA_AT)) * (float)r->div * (1000.0f / 44117.647f)); }
