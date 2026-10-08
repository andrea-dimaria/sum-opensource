/**
 * Loopback test of JanusWrapper, no hardware: the samples modulated by one
 * wrapper are fed to the demodulator of another one (ideal channel).
 *
 * Covers class 11 / app type 2 (text messages, plugin_011_02), class 16
 * regression, ADB fields, RX filters, RX metadata and long packets.
 *
 * usage: janus_loopback_test [--fs N] [--long]
 *   --fs N   sampling frequency (default 96000)
 *   --long   also test 478/480 byte packets (about 49 s of signal each,
 *            takes a few minutes)
 *
 * libplugin_011_02.so and libplugin_016_00.so must be in LD_LIBRARY_PATH.
 */

#include <janus-wrapper.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <thread>

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
