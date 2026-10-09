# JANUS tests (PC, no hardware)

These tests check the JANUS changes (class 11 and class 0 plugins, ADB fields, RX filters, TX length, emergency messages, medium access) without a modem.
- **What they need:** a PC with `cmake`, `gcc`/`g++` and FFTW3.
- **What they don't use:** ALSA, GPIO and the acoustic channel.
- **Not part of the SuM build:** they are not in `Makefile.am`, so `installer.sh` on the modem ignores them.

| File | What it tests |
|---|---|
| `plugin_011_02_test.c` | Class 11 plugin unit test (loaded with `dlopen`) |
| `plugin_000_01_test.c` | Class 0 / app type 1 (Emergency Position) plugin unit test (loaded with `dlopen`) |
| `janus_mac_test.cpp` | `JanusMac` unit test (ANEP-87 §2.2 CSMA/GA and backoff), synthetic channel |
| `janus_loopback_test.cpp` | `JanusWrapper` end to end, using an ideal channel: the samples one wrapper modulates are demodulated by another |
| `run_tests.sh` | Builds libjanus and the plugins into `tests/build/`, then builds and runs all the tests |

## What is checked

**`plugin_011_02_test.c`**
- encode→decode round trip for every text length 1-478;
- the cargo-length steps (32/34, 96/100, 224/232, 480);
- 479 bytes rejected;
- a corrupted CRC or corrupted text detected;
- station 255 rejected;
- the default destination is broadcast.

**`plugin_000_01_test.c`** (Annex D Tables IV-V)
- ADB bit positions (station, destination, cargo size 14), the reservation is required;
- the cargo bits computed by hand: nationality, latitude, longitude, depth, speed, heading, padding, CRC16;
- encode→decode round trip;
- "not available" codes when only the position is given;
- limits (±90°, ±180°, depth and speed saturation, heading 359.99° → 0);
- invalid fields refused; a corrupted cargo detected.

**`janus_mac_test.cpp`** (band A timings: chip 6.25 ms, slot 1.1 s, sensing 2.2 s)
- P(C) = 2/3, 1/2, 1/3, 1/5; which bands overlap the underwater telephone band;
- an idle channel: transmit after 352 chips of sensing, again after each own transmission;
- after a busy window: transmissions only at slot starts, never in the first slot, never within 176 chips;
- the first backoff draw transmits with probability 2/3 (20000 runs);
- a channel that stays busy: wait, no drop; busy in every slot: drop when C reaches 8;
- a received reservation blocks until its end plus the guard time.

**`janus_loopback_test.cpp`** (band D)
- **Startup:** class 11 app type 1 (no plugin) is refused.
- **Class 11:**
  - broadcast delivers the exact text;
  - the RX metadata prefix is correct;
  - own packets are dropped;
  - the class filter works;
  - unicast is accepted by the right station and dropped by others;
  - `0x00` and 479-byte payloads are refused;
  - a 100-byte message gets through.
- **Class 16 regression:** the output is identical to before the changes.
- **Emergency messages:**
  - `SOS` prefix recognition and removal;
  - a position sent as class 0 / app type 1 and received as `SOS lat lon depth speed heading nationality`, also through a class filter;
  - a position with only latitude and longitude;
  - `SOS <text>` sent as a normal message without the prefix.
- **Channel events for the MAC** (noise, a class 11 packet, noise): stationary noise is clear, the packet is busy, and its reservation covers it.
- **With `--long`:** 478 bytes in class 11 and 480 bytes in class 16 (about 49 s of signal each). This checks the TX length fix.

## Usage
```bash
tests/run_tests.sh <libjanus c dir> [--fs N] [--long]
# e.g. from the SuM repo root, with janus-c cloned next to it:
tests/run_tests.sh ../janus-c
tests/run_tests.sh ../janus-c --fs 192000 --long
```

`run_tests.sh` prints `ALL TESTS PASSED` and returns 0, or lists the `FAIL` lines and returns non-zero. libjanus's packet dumps go to `tests/build/loopback_stderr.log`.

## Limitations
- **The channel is ideal:** no multipath, Doppler or collisions (only the channel events test adds white noise).
- **The MAC timing in `janus_executable` (threads, sound card) isn't tested on the PC:** that needs the modems.
- **Wake-up tones aren't tested**, because their reception isn't supported yet.
- **The `janus_executable` program itself isn't exercised:** that means CLI parsing, TCP and threads.
- **Known intermittent failure at 192 kHz:** about one run in three to seven, one random packet is not received (no read error, no CRC error: the demodulator never returns it). It happens with both janus-c and the CMRE reference library, and doesn't go away with slower feeding or longer trailing silence. 96 kHz has not shown it. Still to investigate: it could also drop packets on the modem.
