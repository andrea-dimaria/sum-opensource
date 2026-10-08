# JANUS tests (PC, no hardware)

These tests check the JANUS changes (class 11 plugin, ADB fields, RX filters, TX length) without a modem.
- **What they need:** a PC with `cmake`, `gcc`/`g++` and FFTW3.
- **What they don't use:** ALSA, GPIO and the acoustic channel.
- **Not part of the SuM build:** they are not in `Makefile.am`, so `installer.sh` on the modem ignores them.

| File | What it tests |
|---|---|
| `plugin_011_02_test.c` | Plugin unit test (loaded with `dlopen`) |
| `janus_loopback_test.cpp` | `JanusWrapper` end to end, using an ideal channel: the samples one wrapper modulates are demodulated by another |
| `run_tests.sh` | Builds libjanus and the plugins into `tests/build/`, then builds and runs both tests |

## What is checked

**`plugin_011_02_test.c`**
- encode→decode round trip for every text length 1-478;
- the cargo-length steps (32/34, 96/100, 224/232, 480);
- 479 bytes rejected;
- a corrupted CRC or corrupted text detected;
- station 255 rejected;
- the default destination is broadcast.

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
- **The channel is ideal:** no noise, multipath, Doppler or collisions.
- **Wake-up tones aren't tested**, because their reception isn't supported yet.
- **The `janus_executable` program itself isn't exercised:** that means CLI parsing, TCP and threads.
- **Known intermittent failure at 192 kHz:** about one run in three to seven, one random packet is not received (no read error, no CRC error: the demodulator never returns it). It happens with both janus-c and the CMRE reference library, and doesn't go away with slower feeding or longer trailing silence. 96 kHz has not shown it. Still to investigate: it could also drop packets on the modem.
