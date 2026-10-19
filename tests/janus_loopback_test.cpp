/**
 * Loopback test of JanusWrapper, no hardware: the samples modulated by one
 * wrapper are fed to the demodulator of another one (ideal channel).
 *
 * Covers class 11 / app type 2 (text messages, plugin_011_02), class 16
 * regression, ADB fields, RX filters, RX metadata, long packets, emergency
 * messages (class 0 / app type 1, plugin_000_01) and the channel events
 * given to the MAC (carrier sensing, received reservations).
 *
 * usage: janus_loopback_test [--fs N] [--long]
 *   --fs N   sampling frequency (default 96000)
 *   --long   also test 478/480 byte packets (about 49 s of signal each,
 *            takes a few minutes)
 *
 * libplugin_000_01.so, libplugin_011_02.so and libplugin_016_00.so must be
 * in LD_LIBRARY_PATH.
 */

#include <janus-wrapper.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <random>
#include <mutex>
#include <vector>
#include <sstream>

static int fs = 96000;
static int failures = 0;

struct Node { int cls, app, station, dest; };

static void check(bool ok, const char* name)
{
  printf("%-52s %s\n", name, ok ? "PASS" : "FAIL");
  if(!ok) {
    ++failures;
  }
}

static void config(JanusWrapper& j, const Node& n)
{
  j.setSamplingFrequency(fs);
  j.setCarrierFrequency(14080); // band D
  j.setBandwidth(4160);
  j.setClassId(n.cls);
  j.setAppType(n.app);
  j.setStationId(n.station);
  j.setDestId(n.dest);
}

static std::shared_ptr<Samples> txPacket(const Node& n, const std::string& msg)
{
  JanusWrapper tx;
  config(tx, n);
  if(!tx.janusSetup()) {
    return nullptr;
  }
  auto ck = std::make_shared<Chunk>(600);
  memcpy(ck->data(), msg.data(), msg.size());
  ck->setSize(msg.size());
  if(!tx.modulate(ck)) {
    return nullptr;
  }
  return ck->getModulatedSamples();
}

static std::shared_ptr<Chunk> rxPacket(std::shared_ptr<Samples> s, const Node& n,
  bool metadata, int class_filter = -1)
{
  JanusWrapper rx;
  config(rx, n);
  auto buf = std::make_shared<ConcBuffer>();
  rx.setRxBuffer(buf);
  rx.setSelfFilter(true);
  rx.setRxMetadata(metadata);
  rx.setRxFilter(class_filter, -1);
  if(!rx.janusSetup()) {
    return nullptr;
  }
  // silence, packet, silence, fed in blocks like the sound card does
  std::vector<int16_t> all(fs / 2, 0);
  all.insert(all.end(), s->samples.begin(), s->samples.end());
  all.insert(all.end(), fs, 0);
  const size_t block = 4000;
  for(size_t off = 0; off < all.size(); off += block) {
    size_t k = std::min(block, all.size() - off);
    auto blk = std::make_shared<Samples>(k);
    std::copy(all.begin() + off, all.begin() + off + k, blk->samples.begin());
    blk->sample_size = k;
    rx.demodulate(blk);
    // the RX fifo is finite: feed at most 4x faster than real time, so the
    // demodulator keeps up whatever the sampling frequency
    std::this_thread::sleep_for(std::chrono::microseconds(
      (long)(k * 1e6 / fs / 4)));
  }
  std::shared_ptr<Chunk> out;
  if(buf->wTPop(out, std::chrono::milliseconds(8000))) {
    return out;
  }
  return nullptr;
}

/** modulate a user message as janus_executable does, SOS handling included */
static std::shared_ptr<Samples> txMessage(const Node& n, const std::string& msg,
  bool* emergency = nullptr, bool* position = nullptr)
{
  JanusWrapper tx;
  config(tx, n);
  if(!tx.janusSetup()) {
    return nullptr;
  }
  auto ck = std::make_shared<Chunk>(600);
  memcpy(ck->data(), msg.data(), msg.size());
  ck->setSize(msg.size());
  bool e = JanusWrapper::emergency(ck);
  bool p = e && tx.modulatePosition(ck);
  if(emergency) *emergency = e;
  if(position) *position = p;
  if(!p && !tx.modulate(ck)) {
    return nullptr;
  }
  return ck->getModulatedSamples();
}

/** the numbers of "SOS lat lon depth speed heading nat" within tolerance */
static bool sosEquals(std::shared_ptr<Chunk> c, const char* expected)
{
  if(!c) {
    printf("    received nothing\n");
    return false;
  }
  std::string got(c->data(), c->getSize());
  std::istringstream g(got), e(expected);
  std::string a, b;
  bool ok = true;
  while(ok && (e >> b)) {
    ok = (bool)(g >> a);
    char* end = nullptr;
    double vb = strtod(b.c_str(), &end);
    ok = ok && ((*end == 0 && b != "-") ? std::fabs(strtod(a.c_str(), nullptr) - vb) < 1e-4 : a == b);
  }
  ok = ok && !(g >> a);
  if(!ok) {
    printf("    received '%s'\n", got.c_str());
  }
  return ok;
}

static bool equals(std::shared_ptr<Chunk> c, const std::string& expected)
{
  bool ok = c && c->getSize() == expected.size()
    && memcmp(c->data(), expected.data(), expected.size()) == 0;
  if(!ok) {
    // show what was actually received
    if(!c) {
      printf("    received nothing\n");
    } else {
      printf("    received %zu bytes:", c->getSize());
      for(size_t i = 0; i < c->getSize() && i < 16; ++i) {
        printf(" %02x", (unsigned char)c->data()[i]);
      }
      printf("\n");
    }
  }
  return ok;
}

static std::string text(size_t len)
{
  std::string s(len, 'a');
  for(size_t i = 0; i < len; ++i) {
    s[i] = 'a' + i % 26;
  }
  return s;
}

int main(int argc, char* argv[])
{
  bool long_tests = false;
  for(int i = 1; i < argc; ++i) {
    if(!strcmp(argv[i], "--fs") && i + 1 < argc) {
      fs = atoi(argv[++i]);
    } else if(!strcmp(argv[i], "--long")) {
      long_tests = true;
    } else {
      printf("usage: %s [--fs N] [--long]\n", argv[0]);
      return 2;
    }
  }
  printf("band D, fs = %d Hz\n", fs);

  const Node a11{11, 2, 5, 255}, a11_to9{11, 2, 5, 9};
  const Node b11{11, 2, 9, 255}, c11{11, 2, 7, 255};
  const Node a16{16, 0, 5, 255}, b16{16, 0, 9, 255};

  // startup codec check
  {
    JanusWrapper no_plugin;
    config(no_plugin, Node{11, 1, 5, 255});
    check(!no_plugin.janusSetup(), "class 11 app 1 (no plugin): setup refused");
  }

  // class 11, broadcast
  auto s = txPacket(a11, "hello");
  check(s != nullptr, "class 11: modulate 'hello'");
  if(s) {
    check(equals(rxPacket(s, b11, false), "hello"),
      "class 11: 5 -> broadcast, rx 9, exact text");
    const char md[] = {5, (char)0xff, 0, 0, 11, 2, 2}; // station, dest, pset, class, app, flags (tx_rx)
    check(equals(rxPacket(s, b11, true), std::string(md, sizeof(md)) + "hello"),
      "class 11: rx metadata prefix");
    check(rxPacket(s, a11, false) == nullptr, "class 11: own packet dropped");
    check(rxPacket(s, b11, false, 17) == nullptr, "class 11: class filter 17 drops");
  }

  // class 11, unicast
  auto su = txPacket(a11_to9, "hello 9");
  check(su != nullptr, "class 11: modulate unicast 5 -> 9");
  if(su) {
    check(equals(rxPacket(su, b11, false), "hello 9"), "class 11: 5 -> 9, rx 9 accepts");
    check(rxPacket(su, c11, false) == nullptr, "class 11: 5 -> 9, rx 7 drops");
  }

  // class 11, invalid payloads
  check(txPacket(a11, std::string("bin\0ary", 7)) == nullptr,
    "class 11: payload with 0x00 refused");
  check(txPacket(a11, text(479)) == nullptr, "class 11: 479 bytes refused");

  // class 11, longer message
  auto s100 = txPacket(a11, text(100));
  check(s100 && equals(rxPacket(s100, b11, false), text(100)),
    "class 11: 100 bytes");

  // class 16 regression: raw cargo with plugin padding, as before
  auto s16 = txPacket(a16, "hello");
  check(s16 && equals(rxPacket(s16, b16, false), std::string("hello\0\0\0", 8)),
    "class 16: 'hello' (8 bytes with padding, unchanged)");
  if(s16) {
    // the Station ID travels in the class 16 ADB: the field labels match the plugin
    const char md16[] = {5, (char)0xff, 0, 0, 16, 0, 2}; // no Destination ID in class 16
    check(equals(rxPacket(s16, b16, true), std::string(md16, sizeof(md16))
      + std::string("hello\0\0\0", 8)), "class 16: rx metadata, Station ID 5");
    check(rxPacket(s16, a16, false) == nullptr, "class 16: own packet dropped");
  }

  // TX gain: 0 transmits silence (it used to keep the default amplitude 0.95)
  {
    auto peak = [&](float gain, bool& ok) {
      JanusWrapper tx;
      config(tx, a16);
      ok = tx.setTxGain(gain) && tx.janusSetup();
      auto ck = std::make_shared<Chunk>(100);
      memcpy(ck->data(), "hello", 5);
      ck->setSize(5);
      ok = ok && tx.modulate(ck) && ck->getModulatedSamples();
      int p = 0;
      if(ok) {
        auto s = ck->getModulatedSamples();
        for(size_t i = 0; i < s->sample_size; ++i) {
          p = std::max(p, std::abs((int)s->samples[i]));
        }
      }
      return p;
    };
    bool ok0, ok1, ok9;
    int p0 = peak(0.0f, ok0), p1 = peak(0.1f, ok1), p9 = peak(0.95f, ok9);
    check(ok0 && p0 == 0, "TX gain 0: packet of silence");
    check(ok1 && ok9 && p1 > 0 && std::fabs(p9 / (double)p1 - 9.5) < 0.5,
      "TX gain 0.1 / 0.95: peak in proportion");
    JanusWrapper g;
    check(!g.setTxGain(-0.1f) && !g.setTxGain(1.5f), "TX gain -0.1, 1.5 refused");
  }

  // frequencies: invalid values refused, the previous ones kept
  {
    JanusWrapper w;
    check(!w.setSamplingFrequency(0) && !w.setSamplingFrequency(-96000)
      && w.setSamplingFrequency(fs), "fs 0, -96000 refused");
    check(!w.setCarrierFrequency(0) && !w.setCarrierFrequency(-14080)
      && w.setCarrierFrequency(14080), "center frequency 0, -14080 refused");
    check(!w.setBandwidth(0) && !w.setBandwidth(20000) && w.setBandwidth(4160),
      "bandwidth 0 and above the center frequency refused");
    check(w.janusSetup() && std::fabs(w.chipDuration() - 1 / 160.0) < 1e-9,
      "after refused values: band D (chip 6.25 ms)");
  }

  // emergency messages
  {
    auto chunk = [](const std::string& m) {
      auto c = std::make_shared<Chunk>(100);
      memcpy(c->data(), m.data(), m.size());
      c->setSize(m.size());
      return c;
    };
    auto c1 = chunk("SOS   help me");
    check(JanusWrapper::emergency(c1) && std::string(c1->data(), c1->getSize()) == "help me",
      "SOS: prefix and blanks removed");
    check(!JanusWrapper::emergency(chunk("SOSX")) && !JanusWrapper::emergency(chunk("sos 1 2"))
      && !JanusWrapper::emergency(chunk(" SOS")), "SOS: only exact 'SOS' + blank");
    auto c2 = chunk("SOS");
    check(JanusWrapper::emergency(c2) && std::string(c2->data(), c2->getSize()) == "SOS",
      "SOS alone: emergency, text kept");

    bool e = false, p = false;
    auto sp = txMessage(a16, "SOS 45.5 -1.25 37 2.3 90 IT", &e, &p);
    check(sp && e && p, "SOS position: class 0 / app 1 modulated");
    if(sp) {
      check(sosEquals(rxPacket(sp, b16, false, 16), "SOS 45.5 -1.25 37 2.3 90.2 IT"),
        "SOS position: received, class filter bypassed");
      auto md = rxPacket(sp, b16, true);
      check(md && md->getSize() > 7 && md->data()[0] == 5 && (unsigned char)md->data()[1] == 255
        && md->data()[4] == 0 && md->data()[5] == 1, "SOS position: rx metadata class 0 app 1");
      check(rxPacket(sp, a16, false) == nullptr, "SOS position: own packet dropped");
    }
    auto sp2 = txMessage(a11, "SOS -33.856 151.215", &e, &p);
    check(sp2 && p && sosEquals(rxPacket(sp2, b11, false), "SOS -33.856 151.215 - - - -"),
      "SOS position only, from a class 11 node");
    auto st = txMessage(a11, "SOS help me", &e, &p);
    check(st && e && !p && equals(rxPacket(st, b11, false), "help me"),
      "SOS text: sent as a normal message without prefix");
    txMessage(a16, "SOS 91 0", &e, &p);
    check(e && !p, "SOS latitude 91: not a position");
  }

  // channel events for the MAC: noise, class 11 packet (with reservation), noise
  {
    JanusWrapper rx;
    config(rx, b11);
    auto buf = std::make_shared<ConcBuffer>();
    rx.setRxBuffer(buf);
    std::mutex m;
    std::vector<JanusWrapper::ChannelEvent> events;
    rx.setChannelListener([&](const JanusWrapper::ChannelEvent& ev) {
      std::lock_guard<std::mutex> lk(m);
      events.push_back(ev);
    });
    check(rx.janusSetup(), "events: setup");
    auto pk = txPacket(a11, "hello");
    const double t0 = 3.0; // packet start
    std::mt19937 gen(1);
    std::normal_distribution<double> noise(0.0, 30.0);
    std::vector<int16_t> all((size_t)(t0 * fs), 0);
    if(pk) {
      all.insert(all.end(), pk->samples.begin(), pk->samples.end());
    }
    all.insert(all.end(), (size_t)(4.0 * fs), 0);
    for(auto& v : all) {
      v = (int16_t)std::max(-32768.0, std::min(32767.0, v + noise(gen)));
    }
    const size_t block = 4000;
    for(size_t off = 0; off < all.size(); off += block) {
      size_t k = std::min(block, all.size() - off);
      auto blk = std::make_shared<Samples>(k);
      std::copy(all.begin() + off, all.begin() + off + k, blk->samples.begin());
      blk->sample_size = k;
      rx.demodulate(blk);
      std::this_thread::sleep_for(std::chrono::microseconds((long)(k * 1e6 / fs / 4)));
    }
    std::shared_ptr<Chunk> out;
    bool received = buf->wTPop(out, std::chrono::milliseconds(8000));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::lock_guard<std::mutex> lk(m);
    double len = rx.packetDuration(7); // "hello" + CRC16, class 11 cargo
    int busy_noise = 0, busy_packet = 0, clear = 0, reservations = 0;
    double from = 0, to = 0, last = -1;
    bool monotonic = true;
    for(const auto& ev : events) {
      monotonic = monotonic && ev.time >= last;
      last = ev.time;
      if(ev.time > 1.0 && ev.time < t0 - 0.05) {
        busy_noise += ev.busy;
        clear += !ev.busy;
      }
      if(ev.time > t0 + 0.2 && ev.time < t0 + len - 0.1) {
        busy_packet += ev.busy;
      }
      if(ev.reserved_to > ev.reserved_from) {
        ++reservations;
        from = ev.reserved_from;
        to = ev.reserved_to;
      }
    }
    printf("  %zu events, last %.2f s; noise: %d clear %d busy; packet: %d busy; "
      "reservation %.2f-%.2f s (packet %.2f-%.2f s)\n", events.size(), last, clear,
      busy_noise, busy_packet, from, to, t0, t0 + len);
    check(received && monotonic && last > t0 + len + 2, "events: packet received, time monotonic");
    check(clear > 100 && busy_noise == 0, "events: stationary noise is clear");
    check(busy_packet > 50, "events: busy during the packet");
    check(reservations == 1 && std::fabs(from - t0) < 0.25 && to >= t0 + len - 0.25,
      "events: class 11 reservation covers the packet");
  }

  if(long_tests) {
    auto l11 = txPacket(a11, text(478));
    check(l11 && equals(rxPacket(l11, b11, false), text(478)),
      "class 11: 478 bytes (longest text)");
    auto l16 = txPacket(a16, text(480));
    auto r16 = l16 ? rxPacket(l16, b16, false) : nullptr;
    check(r16 && r16->getSize() == 480 && !memcmp(r16->data(), text(480).data(), 480),
      "class 16: 480 bytes (longest cargo)");
  }

  printf(failures ? "%d FAILURE(S)\n" : "ALL TESTS PASSED\n", failures);
  return failures != 0;
}
