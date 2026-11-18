# USB AUDIO IN and USB REC (fork)

What a computer or a phone plays to the FM-1 over its USB-C port: heard through the FM-1's mixer and effects, and/or
recorded into one of your sample slots (USR1..USR4). The FM-1 stays a USB *device*; the phone or computer is the host.
(Plugging an audio interface *into* the FM-1 would need the FM-1 to be a USB host; this does not do that.)

**Status: not tested on a real FM-1.** The PC parts (the ring buffer and converter, the ADPCM recorder, the whole
firmware on the PC twin) are tested and pass. The USB endpoint, the descriptors and the controller registers are written
from the other endpoints' code in `hal/fm1/usb.c`; the firmware itself has not been built here (no pi32v2 toolchain).

## Using it

HOME held + SELECT, page 5 (**USB IN 5/5**):

* `USB IN` ON/OFF (KNOB 1), `LEVEL` 0-100 % (KNOB 2), `STREAM` (KNOB 3, read only): LIVE when the host's audio is
  arriving, with the receive buffer's fill as its bar.

The audio comes in mono ((L + R) / 2: the RAM is tight) and is mixed in dry, before the master bus, so the punch-ins, the
DJ filter and the limiter work on it like on a track. The settings are kept with the project (HOME bytes 77-78).

## USB REC (off in the default build)

The firmware's RAM is almost full, so recording the incoming audio into a sample slot is behind a build flag:
`USB_REC=1 sh fw/build.sh`. With it HOME gets a page 6 (**USB REC 6/6**): `SLOT` (USR1-4), `LENGTH` (3.7 s, 7.4 s or
14.8 s) and `REC` (KNOB 3 right to start, left to stop). REC erases the slot (its old sample is lost), says `READY, PLAY
NOW`, waits for a sound (-50 dBFS) and records until you stop it or the slot is full. The sectors are written as they fill
and the header last: a cut leaves the slot empty, never half a sample. The take is mono IMA ADPCM in the slot's "FSMP"
layout, so SAMPLE, SLICE and YOUR KITS play it. It needs a few KB more RAM than the default build has: it is tested on
the PC twin (`make build/twin`, built with `-DUSB_REC`) but may not link for the FM-1 until more RAM is found.

## What changed

| file | change |
| --- | --- |
| `hal/fm1/usb.c` | a USB audio *output*: interface 3, isochronous OUT endpoint 2 (16-bit stereo, 44.1/48 kHz, SET_CUR on the endpoint), packets to `uin_push`; device 3.20 |
| `app/usbin.[ch]` | each packet converted at once, in integers (the tick must not touch the float registers), by linear interpolation to the engine's rate into a 448-sample mono ring; the step follows the ring's fill (the host's and the FM-1's clocks differ) |
| `engine/sound.c`, `engine.h` | the USB IN bus: dry into the mix, before the master bus |
| `app/store.c` | the CRC-32 by nibbles (a 64-byte table) instead of a 1 KB table in RAM: the room for the above. Same results. |
| `fw/main.c` | the HOME page, the pull each block (into usb_l's block, which snd_mix writes only after reading it), the settings; with `USB_REC` the recorder |
| `app/usbrec.[ch]` | (USB_REC) the recorder: ADPCM encoder (the SAMPLE engine's decoder inverted), the sector buffers, the slot header |
| `tests/usbin_test.c`, `tests/usbrec_test.c`, `host/twin.c` (`usbin`, `usbrec`) | the tests |

## Known limits

* USB IN has no send to the delay or reverb: it is dry (the master bus effects do reach it). It is mono.
* If a computer records the FM-1's USB output while also playing to it, the audio loops: keep them apart.
* The ring takes the engine's burst of four blocks and the main loop up to 1.5 ms late (tested on the PC with clock
  errors of up to +-800 ppm): a main loop later than that glitches.
