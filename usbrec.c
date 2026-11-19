/* SPDX-License-Identifier: GPL-3.0-only
 * USB REC (usbrec.h). The encoder is IMA ADPCM as engine/sample.c decodes it (low nibble first, the same step and
 * index tables, the predictor rebuilt exactly as the decoder rebuilds it, so what is heard is what was encoded). */
#include "usbrec.h"
#include "../engine/sample.h"
#if defined(__PI32V2__)
#define XT __attribute__((section(".xtext.usbrec")))   /* from the flash window, not the RAM: it runs only between the flash's jobs */
#else
#define XT
#endif

XT static uint32_t crc_upd(uint32_t crc, uint8_t b)         /* CRC-32 (IEEE, as store.c's crc32_ieee) one byte at a time */
{
    crc = ~crc ^ b;
    for (int i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    return ~crc;
}

XT void urec_begin(usbrec *r, uint8_t *buf, uint32_t slot_size, int div)
{
    uint8_t *p = (uint8_t *)r;
    for (uint32_t i = 0; i < sizeof *r; i++) p[i] = 0;
    r->buf = buf; r->size = slot_size; r->div = (uint8_t)(div == 2 ? 2 : div == 4 ? 4 : 1);
    r->sent = r->wr = UREC_DATA_AT;
    for (uint32_t i = 0; i < UREC_BUF; i++) buf[i] = 0xFF;
    r->on = 1;
}

XT static int encode(usbrec *r, int s)                       /* one sample (16 bits) to a code of 4 bits */
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

XT static void put_byte(usbrec *r, uint8_t b)
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
    if (UREC_DATA_AT + r->bytes >= r->size) { r->full = 1; r->on = 0; }
}

XT static void put_sample(usbrec *r, int s)
{
    int code = encode(r, s);
    r->samples++;
    if (!r->odd) { r->lo = (uint8_t)code; r->odd = 1; }
    else { r->odd = 0; put_byte(r, (uint8_t)(r->lo | code << 4)); }
}

XT void urec_feed(usbrec *r, const int16_t *m, uint32_t n)
{
    for (uint32_t i = 0; i < n && r->on; i++) {
        int v = m[i];
        if (!r->started) { if (v < UREC_THRESH && v > -UREC_THRESH) continue; r->started = 1; }   /* armed: waiting for the sound */
        r->acc += v;
        if (++r->cnt < r->div) continue;
        int mean = r->acc >> (r->div >> 1);                 /* (the mean of div samples: div is 1, 2 or 4) */
        r->acc = 0; r->cnt = 0;
        put_sample(r, mean);
    }
}

XT const uint8_t *urec_take(usbrec *r, uint32_t *sector, uint32_t *from, uint32_t *n)
{
    uint32_t made = UREC_DATA_AT + r->bytes, page_end = (r->sent / UREC_PAGE + 1u) * UREC_PAGE;
    if (made < page_end && !(r->stopped && made > r->sent)) return 0;       /* a page is not complete yet */
    uint32_t end = made < page_end ? made : page_end;
    if (end <= r->sent) return 0;
    uint32_t sec = r->sent / UREC_SECTOR, f = r->sent % UREC_SECTOR;
    *sector = sec; *from = f; *n = end - r->sent;
    const uint8_t *d = r->buf + (sec & 1u) * UREC_SECTOR + f;
    r->sent = end;
    return d;
}

XT void urec_written(usbrec *r, uint32_t n)
{
    r->wr += n;
    if (r->wr % UREC_SECTOR == 0) r->busy[(r->wr / UREC_SECTOR - 1u) & 1u] = 0;       /* a sector complete: its half is free */
    if (r->stopped && r->wr >= UREC_DATA_AT + r->bytes) r->busy[0] = r->busy[1] = 0;
}

XT uint32_t urec_stop(usbrec *r)
{
    if (r->odd) { r->odd = 0; put_byte(r, r->lo); }          /* (the last sample alone: its byte's high nibble 0) */
    r->on = 0; r->stopped = 1;
    return r->bytes ? r->samples : 0u;
}

XT int urec_idle(const usbrec *r) { return r->wr >= UREC_DATA_AT + r->bytes && r->sent == r->wr; }

XT static void w32(uint8_t *b, uint32_t v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24); }
XT void urec_header(const usbrec *r, uint8_t *h, const char *name)
{
    for (uint32_t i = 0; i < UREC_HEAD; i++) h[i] = 0;
    w32(h, 0x504D5346u); h[4] = 1; h[5] = 0; h[6] = 1; h[7] = 0;                 /* "FSMP", version 1, one zone */
    for (int i = 0; i < 8 && name && name[i]; i++) h[8 + i] = (uint8_t)name[i];
    w32(h + 16, r->bytes); w32(h + 20, r->crc);
    uint8_t *z = h + 32;                                                       /* the zone: all of it, from C4 up and down */
    w32(z, 0); w32(z + 4, r->samples); w32(z + 8, 0); w32(z + 12, r->samples - 1u); w32(z + 16, 65536u / r->div);
    z[20] = 960 & 0xFF; z[21] = 960 >> 8; z[22] = 0; z[23] = 0; z[24] = 0; z[25] = 0; z[26] = 127; z[27] = 0;
}
XT uint32_t urec_len_ms(const usbrec *r) { return (uint32_t)((float)r->samples * (float)r->div * (1000.0f / 44117.647f)); }
XT uint32_t urec_room_ms(const usbrec *r) { return (uint32_t)((float)(2u * (r->size - UREC_DATA_AT)) * (float)r->div * (1000.0f / 44117.647f)); }
