/* SPDX-License-Identifier: GPL-3.0-only
 * USB REC (usbrec.c): the audio that comes in over USB (usbin.h) recorded into one of your sample slots (USR1..USR4),
 * as IMA ADPCM in SLOOP's "FSMP" layout (engine/sample.h), mono, at the engine's rate or half or a quarter of it.
 * All of it runs from the flash window (the main loop's take handling), none in the RAM: the firmware's RAM is nearly full.
 * That means it cannot run while the flash works, so the flash is written a page (256 bytes, about a millisecond) at a time
 * and the USB ring (usbin.h) keeps the audio meanwhile. The module only encodes and keeps the sector buffers; fw/main.c
 * writes the pieces to the flash (store_raw_begin) as they fill and the header last, so a cut at any point leaves the slot
 * as it was (no header, no sample). No hardware here: it runs on the PC (tests/usbrec_test.c). Written for this project. */
#pragma once
#include <stdint.h>

#define UREC_SECTOR 4096u
#define UREC_PAGE 256u
#define UREC_DATA_AT 512u                  /* the data begins here in the slot, after the header's 512 bytes */
#define UREC_BUF (2u * UREC_SECTOR)        /* the buffer the caller gives: two sectors */
#define UREC_HEAD 480u                     /* the header's bytes written (the zone table) */
#define UREC_THRESH 100                    /* the signal that starts a take (16-bit: -50 dBFS) */

typedef struct {
    uint8_t *buf;                          /* two sectors: the sector n is in half n & 1 */
    uint32_t size;                         /* the slot's size (SMP_USR_SIZE) */
    uint8_t on, started, full, err, stopped, div, cnt, odd, busy[2];
    int16_t pred; uint8_t idx; uint8_t lo;
    int32_t acc;
    uint32_t bytes, samples, crc;          /* the data bytes made, the samples encoded, their CRC-32 */
    uint32_t sent, wr;                     /* the slot's offsets (data begins at UREC_DATA_AT): handed to the flash, written by it */
} usbrec;

enum { UREC_OK = 0, UREC_ERR_OVERRUN = 1 };   /* err: a sector was not written before its half was needed again */

/* div: 1, 2 or 4 (the rate recorded: all, a half, a quarter of the engine's) */
void urec_begin(usbrec *r, uint8_t *buf, uint32_t slot_size, int div);
/* mono samples (16 bits) of the incoming audio: until one passes the threshold nothing is kept (armed) */
void urec_feed(usbrec *r, const int16_t *m, uint32_t n);
/* the next page (or the last, shorter piece) ready for the flash: its sector, the offset in it, the bytes, where they are
 * (0: none yet). The pieces follow each other; write one, call urec_written, then ask for the next. */
const uint8_t *urec_take(usbrec *r, uint32_t *sector, uint32_t *from, uint32_t *n);
void urec_written(usbrec *r, uint32_t n);           /* the piece of n bytes is in the flash: a sector done frees its half */
/* stops the take: the last piece is made ready (urec_take gives it); returns the samples recorded (0: nothing) */
uint32_t urec_stop(usbrec *r);
/* the header's UREC_HEAD bytes, once every piece is written */
void urec_header(const usbrec *r, uint8_t *h, const char *name);
int urec_idle(const usbrec *r);                     /* every byte made has been written */
uint32_t urec_len_ms(const usbrec *r);              /* the take's length so far, ms */
uint32_t urec_room_ms(const usbrec *r);             /* the room the slot has for a take at this rate, ms */
