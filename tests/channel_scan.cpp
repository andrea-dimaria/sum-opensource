/**
 * Channel scan of a recording (PC tool, no hardware): the WAV is given to a
 * JanusWrapper receiver as the sound card would, and the channel events
 * given to the MAC are analysed.
 *
 * Reports:
 * - busy time and busy intervals (ANEP-87 §2.2.1 energy detector of libjanus,
 *   plus preamble detections), separating busy from energy only and busy
 *   with a preamble detection;
 * - decoded packets;
 * - a replay of the MAC (JanusMac) on the recording: a packet requested every
 *   --request-every seconds, with the delay before transmitting and the drops.
 * On a noise-only recording every busy interval is a false alarm.
 *
 * usage: channel_scan <wav> [--center-freq N] [--bandwidth N]
 *          [--detection-threshold X] [--speed X] [--request-every S]
 *          [--seed N] [--csv FILE]
 *   wav        PCM 16 bit, any rate (the rate of the modem, e.g. 192000);
 *              with more channels only the first is used
 *   --center-freq, --bandwidth   JANUS band (default 11520 / 4160, band A)
 *   --detection-threshold        preamble detection threshold (default 3.0)
 *   --speed X  feeding speed relative to real time (default 4): the receiver
 *              must keep up, lower it if packets are missed
 *   --request-every S   MAC replay request period (default 10 s)
 *   --csv FILE every event: time, busy, detected, window_power, background_power
 *
 * The libjanus plugins must be in LD_LIBRARY_PATH (decoded packets).
 */

#include <janus-wrapper.h>
#include <janus-mac.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/** read a PCM 16 bit WAV, first channel only */
static bool readWav(const char* path, std::vector<int16_t>& out, int& fs)
{
  std::ifstream f(path, std::ios::binary);
  char riff[12];
  if(!f.read(riff, 12) || memcmp(riff, "RIFF", 4) || memcmp(riff + 8, "WAVE", 4)) {
    return false;
  }
  int channels = 0, bits = 0;
  char hdr[8];
  while(f.read(hdr, 8)) {
    uint32_t size;
    memcpy(&size, hdr + 4, 4);
    if(!memcmp(hdr, "fmt ", 4)) {
      std::vector<char> fmt(size);
      f.read(fmt.data(), size);
      uint16_t format, ch, bps;
      uint32_t rate;
      memcpy(&format, &fmt[0], 2);
      memcpy(&ch, &fmt[2], 2);
      memcpy(&rate, &fmt[4], 4);
      memcpy(&bps, &fmt[14], 2);
      if(format != 1 && format != 0xFFFE) {
        return false;
      }
      channels = ch;
      fs = rate;
      bits = bps;
    } else if(!memcmp(hdr, "data", 4)) {
      if(channels < 1 || bits != 16) {
        return false;
      }
      std::vector<int16_t> all;
      // arecord leaves the size at its maximum when interrupted: read to EOF
      std::vector<char> buf(1 << 20);
      while(f.read(buf.data(), buf.size()) || f.gcount() > 0) {
        size_t n = f.gcount() / 2;
        size_t old = all.size();
        all.resize(old + n);
        memcpy(&all[old], buf.data(), n * 2);
      }
      out.resize(all.size() / channels);
      for(size_t i = 0; i < out.size(); ++i) {
        out[i] = all[i * channels];
      }
      return true;
    } else {
      f.seekg(size + (size & 1), std::ios::cur);
    }
  }
  return false;
}

struct Interval {
  double from, to;
  bool detected;
};

int main(int argc, char* argv[])
{
  if(argc < 2 || argv[1][0] == '-') {
    printf("usage: %s <wav> [--center-freq N] [--bandwidth N] [--detection-threshold X]\n"
      "  [--speed X] [--request-every S] [--seed N] [--csv FILE]\n", argv[0]);
    return 2;
  }
  const char* wav = argv[1];
  double center = 11520, bandwidth = 4160, threshold = 3.0, speed = 4, every = 10;
  unsigned seed = 1;
  const char* csv = nullptr;
  for(int i = 2; i + 1 < argc; i += 2) {
    std::string o = argv[i];
    if(o == "--center-freq") center = atof(argv[i + 1]);
    else if(o == "--bandwidth") bandwidth = atof(argv[i + 1]);
    else if(o == "--detection-threshold") threshold = atof(argv[i + 1]);
    else if(o == "--speed") speed = atof(argv[i + 1]);
    else if(o == "--request-every") every = atof(argv[i + 1]);
    else if(o == "--seed") seed = atoi(argv[i + 1]);
    else if(o == "--csv") csv = argv[i + 1];
    else {
      printf("unknown option %s\n", argv[i]);
      return 2;
    }
  }

  std::vector<int16_t> samples;
  int fs = 0;
  if(!readWav(wav, samples, fs) || samples.empty()) {
    printf("%s: not a PCM 16 bit WAV\n", wav);
    return 1;
  }
  double duration = (double)samples.size() / fs;
  printf("%s: %.1f s at %d Hz, band %.0f/%.0f Hz, detection threshold %.2f\n",
    wav, duration, fs, center, bandwidth, threshold);

  JanusWrapper rx;
  rx.setSamplingFrequency(fs);
  rx.setCarrierFrequency(center);
  rx.setBandwidth(bandwidth);
  rx.setDetectionThreshold(threshold);
  auto buf = std::make_shared<ConcBuffer>();
  rx.setRxBuffer(buf);
  std::mutex m;
  std::vector<JanusWrapper::ChannelEvent> events;
  rx.setChannelListener([&](const JanusWrapper::ChannelEvent& ev) {
    std::lock_guard<std::mutex> lk(m);
    events.push_back(ev);
  });
  if(!rx.janusSetup()) {
    printf("JANUS setup failed\n");
    return 1;
  }

  // fed in blocks like the sound card does, faster than real time
  const size_t block = 4000;
  auto start = std::chrono::steady_clock::now();
  for(size_t off = 0; off < samples.size(); off += block) {
    size_t k = std::min(block, samples.size() - off);
    auto blk = std::make_shared<Samples>(k);
    std::copy(samples.begin() + off, samples.begin() + off + k, blk->samples.begin());
    blk->sample_size = k;
    rx.demodulate(blk);
    std::this_thread::sleep_until(start + std::chrono::microseconds(
      (long long)((off + k) * 1e6 / fs / speed)));
  }
  int packets = 0;
  std::shared_ptr<Chunk> out;
  while(buf->wTPop(out, std::chrono::milliseconds(2000))) {
    ++packets;
  }
  std::lock_guard<std::mutex> lk(m);

  // busy intervals (consecutive busy events), and the analysed time
  std::vector<Interval> busy;
  double prev = events.empty() ? 0 : events.front().time;
  double busy_time = 0;
  bool in_busy = false;
  for(const auto& ev : events) {
    if(ev.busy) {
      if(!in_busy) {
        busy.push_back({prev, ev.time, ev.detected});
      }
      busy.back().to = ev.time;
      busy.back().detected = busy.back().detected || ev.detected;
      busy_time += ev.time - prev;
    }
    in_busy = ev.busy;
    prev = ev.time;
  }
  double analysed = events.empty() ? 0 : events.back().time - events.front().time;
  int energy_only = 0;
  double energy_time = 0, longest = 0;
  for(const auto& b : busy) {
    if(!b.detected) {
      ++energy_only;
      energy_time += b.to - b.from;
    }
    longest = std::max(longest, b.to - b.from);
  }
  printf("events: %zu, analysed %.1f s (last at %.1f s, recording %.1f s)\n",
    events.size(), analysed, events.empty() ? 0 : events.back().time, duration);
  printf("busy: %zu intervals, %.1f s = %.2f %% of the time, longest %.2f s\n",
    busy.size(), busy_time, analysed > 0 ? 100 * busy_time / analysed : 0, longest);
  printf("  energy only (no preamble): %d intervals, %.1f s; with a preamble detection: %zu\n",
    energy_only, energy_time, busy.size() - energy_only);
  printf("decoded packets: %d\n", packets);
  size_t shown = 0;
  for(const auto& b : busy) {
    if(shown++ == 40) {
      printf("  ... %zu more\n", busy.size() - 40);
      break;
    }
    printf("  %9.3f - %9.3f s  %6.3f s  %s\n", b.from, b.to, b.to - b.from,
      b.detected ? "preamble" : "energy");
  }

  // MAC replay: a request every `every` seconds of receiver time
  JanusMac mac(rx.chipDuration(), JanusMac::overlapsUtBand(center, bandwidth), seed);
  std::vector<double> delays;
  int requests = 0, drops = 0;
  double next = events.empty() ? 0 : events.front().time + every, asked = -1;
  for(const auto& ev : events) {
    if(ev.reserved_to > ev.reserved_from) {
      mac.reserve(ev.reserved_from, ev.reserved_to);
    }
    mac.observe(ev.time, ev.busy);
    if(asked < 0 && ev.time >= next) {
      mac.request();
      asked = ev.time;
      ++requests;
    }
    if(asked >= 0) {
      JanusMac::Decision d = mac.decision();
      if(d == JanusMac::TRANSMIT || d == JanusMac::DROP) {
        if(d == JanusMac::TRANSMIT) {
          delays.push_back(ev.time - asked);
          mac.transmitted();
        } else {
          ++drops;
          mac.cancel();
        }
        asked = -1;
        next = std::max(next + every, ev.time);
      }
    }
  }
  std::sort(delays.begin(), delays.end());
  auto pct = [&](double p) {
    return delays.empty() ? 0.0 : delays[std::min(delays.size() - 1, (size_t)(p * delays.size()))];
  };
  int delayed = std::count_if(delays.begin(), delays.end(), [](double d) { return d > 0.05; });
  printf("MAC replay (request every %.0f s, chip %.2f ms): %d requests, %zu sent, %d dropped, "
    "%d still waiting\n", every, rx.chipDuration() * 1000, requests, delays.size(), drops,
    asked >= 0 ? 1 : 0);
  printf("  delay: %d of %zu delayed by the MAC; median %.2f s, 90 %% %.2f s, max %.2f s\n",
    delayed, delays.size(), pct(0.5), pct(0.9), delays.empty() ? 0 : delays.back());

  if(csv) {
    FILE* f = fopen(csv, "w");
    if(!f) {
      printf("cannot write %s\n", csv);
      return 1;
    }
    fprintf(f, "time,busy,detected,window_power,background_power\n");
    for(const auto& ev : events) {
      fprintf(f, "%.4f,%d,%d,%.6g,%.6g\n", ev.time, ev.busy, ev.detected,
        ev.window_power, ev.background_power);
    }
    fclose(f);
    printf("events written to %s\n", csv);
  }
  return 0;
}
