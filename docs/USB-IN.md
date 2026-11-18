# USB AUDIO IN and USB REC (fork)

What a computer or a phone plays to the FM-1 over its USB-C port: heard through the FM-1's mixer and effects, and/or
recorded into one of your sample slots (USR1..USR4). The FM-1 stays a USB *device*; the phone or computer is the host.
(Plugging an audio interface *into* the FM-1 would need the FM-1 to be a USB host; this does not do that.)

**Status: not tested on a real FM-1.** The PC parts (the ring buffer and resampler, the ADPCM recorder, the whole
firmware on the PC twin) are tested and pass. The USB endpoint, the descriptors and the controller registers are written
from the other endpoints' code in `hal/fm1/usb.c`; the firmware itself has not been built here (no pi32v2 toolchain).

## Using it

HOME held + SELECT, pages 5 and 6:

* **USB IN 5/6** — `USB IN` ON/OFF, `LEVEL` (0-100 %), `STREAM` (LIVE when the host's audio is arriving; the bar is the
  receive buffer's fill).
* **USB REC 6/6** — `SLOT` (USR1-4), `LENGTH` (3.7 s, 7.4 s or 14.8 s: all, half or a quarter of the sample rate),
  `REC`: turn KNOB 3 right to start, left to stop.

REC erases the chosen slot (its old sample is lost), says `READY, PLAY NOW`, then waits for a sound (it starts at the first
audio above -50 dBFS, so the silence before is not kept) and records until you stop it or the slot is full. The sectors are
written as they fill and the header last: a cut at any point leaves the slot empty, never half a sample. The take is mono
(left + right), IMA ADPCM, the slot's "FSMP" layout, so SAMPLE, SLICE and YOUR KITS play it as any user sample.
While the flash works the screen waits and other samples read from the flash are silent (as in a save).

The settings are kept with the project (HOME bytes 77-79).

## What changed

| file | change |
| --- | --- |
| `hal/fm1/usb.c` | a USB audio *output*: interface 3, isochronous OUT endpoint 2 (16-bit stereo, 44.1/48 kHz, SET_CUR on the endpoint), packets to `uin_push`; device 3.20 |
| `app/usbin.[ch]` | the ring (512 frames) and the resampler: linear interpolation, the read step steered by the ring's fill (host and FM-1 clocks differ) |
| `app/usbrec.[ch]` | the recorder: ADPCM encoder (the SAMPLE engine's decoder inverted), the sector buffers, the slot header |
| `engine/sound.c`, `engine.h` | the USB IN bus: dry into the mix, before the master bus (punch-ins, DJ filter, limiter) |
| `fw/main.c` | the HOME pages, the pull each block, the take's flash jobs (the editor's `store_raw_begin`), the settings |
| `tests/usbin_test.c`, `tests/usbrec_test.c`, `host/twin.c` (`usbrec`) | the tests |

## Known limits

* The take goes to the flash as it is made: the slot is erased first (about a second, longer if the flash is slow).
* USB IN has no send to the delay or reverb: it is dry (the master bus effects do reach it).
* If a computer records the FM-1's USB output while also playing to it, the audio loops: keep them apart.
* If the flash is too slow for a take (a sector not written before its buffer is needed), the take stops with
  `REC: FLASH TOO SLOW` and the slot is left empty.
* The receive buffer is 2 KB and the code a few KB more (not measured): if the firmware no longer fits its RAM slot (`fw/ram.ld`), shrink
  `UIN_N` in `app/usbin.h` first.
