/* SPDX-License-Identifier: GPL-3.0-only
 * USB REC (usbrec.c): the audio that comes in over USB (usbin.h) recorded into one of your sample slots (USR1..USR4),
 * as IMA ADPCM in SLOOP's "FSMP" layout (engine/sample.h), mono, at the engine's rate or half or a quarter of it.
 * The module only encodes and keeps the sector buffers; the firmware (fw/main.c) writes the sectors to the flash as
 * they fill (store_raw_begin) and the header last, so a cut at any point leaves the slot as it was (no header, no
 * sample). No hardware here: it runs on the PC (tests/usbrec_test.c). Written for this project. */
#pragma once
#include <stdint.h>

#define UREC_SECTOR 4096u
#define UREC_DATA_AT 512u                  /* the data begins here in the slot, after the header's 512 bytes */
#define UREC_BUF (2u * UREC_SECTOR)        /* the buffer the caller gives: two sectors */
#define UREC_HEAD 480u                     /* the header's bytes written (the zone table) */
#define UREC_THRESH 0.003f                 /* the signal that starts a take (-50 dBFS) */

typedef struct {
    uint8_t *buf;                          /* two sectors: the sector n is in half n & 1 */
    uint32_t size;                         /* the slot's size (SMP_USR_SIZE) */
    uint8_t on, started, full, err, div, cnt, odd, busy[2];
    int16_t pred; uint8_t idx; uint8_t lo;
    float acc;
    uint32_t bytes, samples, crc;          /* the data bytes made, the samples encoded, their CRC-32 */
    uint32_t ready, taken;                 /* the sectors complete (0 .. ready - 1), and the ones handed out by urec_take */
} usbrec;

enum { UREC_OK = 0, UREC_ERR_OVERRUN = 1 };   /* err: a sector was not written before its half was needed again */

/* div: 1, 2 or 4 (the rate recorded: all, a half, a quarter of the engine's) */
void urec_begin(usbrec *r, uint8_t *buf, uint32_t slot_size, int div);
/* a block of the incoming audio (-1..1): until it passes the threshold nothing is kept (armed) */
void urec_feed(usbrec *r, const float *l, const float *rt, uint32_t n);
/* a sector filled and not yet taken: its number, the offset in it to write from, the bytes, where they are (0: none) */
const uint8_t *urec_take(usbrec *r, uint32_t *sector, uint32_t *from, uint32_t *n);
void urec_written(usbrec *r, uint32_t sector);      /* that sector is in the flash: its half is free again */
/* stops the take: the last part sector is made ready (urec_take gives it); returns the samples recorded (0: nothing) */
uint32_t urec_stop(usbrec *r);
/* the header's UREC_HEAD bytes, once every sector is written */
void urec_header(const usbrec *r, uint8_t *h, const char *name);
uint32_t urec_len_ms(const usbrec *r);              /* the take's length so far, ms */
uint32_t urec_room_ms(const usbrec *r);             /* the room the slot has for a take at this rate, ms */
