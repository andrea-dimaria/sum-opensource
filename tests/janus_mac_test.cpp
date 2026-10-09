/**
 * Unit test of JanusMac (ANEP-87 §2.2 CSMA/GA + BEB), no audio: the channel
 * is a function of time, observed every 5 ms like the demodulator does.
 *
 * usage: janus_mac_test
 */

#include <janus-mac.h>
#include <cstdio>
#include <cmath>
#include <functional>

static int failures = 0;

static void check(bool ok, const char* name)
{
  printf("%-52s %s\n", name, ok ? "PASS" : "FAIL");
  if(!ok) {
    ++failures;
  }
}

static const double CD = 26.0 / 4160; // band A chip duration, 6.25 ms
static const double SLOT = 176 * CD; // 1.1 s
static const double SENSE = 352 * CD; // 2.2 s
static const double DT = 0.005;

typedef std::function<bool(double)> Channel;

static bool idle(double) { return false; }

/** observe from t0 to t1, stop at the first decision other than WAIT */
static JanusMac::Decision run(JanusMac& mac, double t0, double t1,
  Channel busy, double* at = nullptr)
{
  for(double t = t0; t <= t1 + 1e-9; t += DT) {
    mac.observe(t, busy(t));
    JanusMac::Decision d = mac.decision();
    if(d != JanusMac::WAIT && d != JanusMac::NONE) {
      if(at) {
        *at = t;
      }
      return d;
    }
  }
  return mac.decision();
}

int main()
{
  check(std::fabs(JanusMac::probability(1) - 2.0 / 3) < 1e-12
    && std::fabs(JanusMac::probability(2) - 1.0 / 2) < 1e-12
    && std::fabs(JanusMac::probability(3) - 1.0 / 3) < 1e-12
    && std::fabs(JanusMac::probability(4) - 1.0 / 5) < 1e-12
    && std::fabs(JanusMac::probability(7) - 1.0 / 5) < 1e-12,
    "P(C) = 2/3, 1/2, 1/3, 1/5, 1/5");
  check(JanusMac::overlapsUtBand(11520, 4160) && JanusMac::overlapsUtBand(9700, 2600)
    && !JanusMac::overlapsUtBand(14080, 4160) && !JanusMac::overlapsUtBand(6000, 2080)
    && !JanusMac::overlapsUtBand(28000, 6500),
    "UT band overlap: A, C yes; B, D, E no");

  {
    JanusMac mac(CD, true, 1);
    mac.observe(0, false);
    check(mac.decision() == JanusMac::NONE, "no packet: NONE");
    mac.request();
    double at = -1;
    JanusMac::Decision d = run(mac, DT, 5, idle, &at);
    check(d == JanusMac::TRANSMIT && at >= SENSE - 1e-9 && at < SENSE + 2 * DT,
      "idle channel: transmit after 352 Cd of sensing");
  }

  {
    JanusMac mac(CD, true, 1);
    run(mac, 0, 3, idle);
    mac.request();
    check(mac.decision() == JanusMac::TRANSMIT, "already sensed, idle: transmit immediately");
    mac.transmitted();
    check(mac.sensed() == 0 && mac.counter() == 1, "after TX: sensing restarts, C = 1");
    mac.request();
    double t0 = mac.now(), at = -1;
    run(mac, t0 + DT, t0 + 5, idle, &at);
    check(at - t0 >= SENSE - 1e-9, "next packet waits 352 Cd after own TX");
  }

  {
    // busy 0.4 s before the request: backoff, never within the guard time,
    // never in the first slot (band A)
    bool ok = true, all_tx = true;
    for(unsigned seed = 1; seed <= 200; ++seed) {
      JanusMac mac(CD, true, seed);
      Channel ch = [](double t) { return t >= 2.5 && t < 2.6; };
      run(mac, 0, 3.0, ch);
      mac.request();
      double at = -1;
      JanusMac::Decision d = run(mac, 3.0 + DT, 60, ch, &at);
      all_tx = all_tx && d == JanusMac::TRANSMIT;
      // channel already clear at the request (3.0): the slots start there;
      // no TX in the first slot (band A) nor within 176 Cd of the busy end
      double k = (at - 3.0) / SLOT;
      ok = ok && at - 2.6 >= SLOT - 1e-9 && k > 0.5
        && std::fabs(k - std::round(k)) < 2 * DT / SLOT;
    }
    check(all_tx, "recent busy: eventually transmits");
    check(ok, "recent busy: TX at slot start, guard and band A kept");
  }

  {
    // P(1): transmit at the second slot start with probability 2/3
    int tx = 0, runs = 20000;
    for(int seed = 1; seed <= runs; ++seed) {
      JanusMac mac(CD, false, seed);
      Channel ch = [](double t) { return t >= 2.5 && t < 2.6; };
      run(mac, 0, 3.0, ch);
      mac.request();
      double at = -1;
      run(mac, 3.0 + DT, 2.6 + 2 * SLOT - DT / 2, ch, &at);
      tx += mac.decision() == JanusMac::TRANSMIT;
    }
    double p = (double)tx / runs;
    printf("  P(transmit in second slot) = %.4f (2/3)\n", p);
    check(std::fabs(p - 2.0 / 3) < 0.015, "BEB first draw has P = 2/3");
  }

  {
    JanusMac mac(CD, true, 7);
    run(mac, 0, 3, idle);
    Channel ch = [](double t) { return t >= 2.9; };
    run(mac, 3, 3, ch);
    mac.request();
    JanusMac::Decision d = run(mac, 3.0, 60, ch);
    check(d == JanusMac::WAIT && mac.counter() == 1,
      "continuously busy: waits for a clear channel, no drop");
  }

  {
    // a short busy window in the middle of every slot: C grows, drop at 8
    bool ok = true;
    for(unsigned seed = 1; seed <= 50; ++seed) {
      JanusMac mac(CD, false, seed);
      Channel ch = [](double t) {
        if(t < 2.9) return false;
        if(t < 3.0) return true;
        double ph = std::fmod(t - 3.0, SLOT);
        return ph >= SLOT / 2 && ph < SLOT / 2 + 0.01;
      };
      run(mac, 0, 2.95, ch);
      mac.request();
      JanusMac::Decision d = run(mac, 2.95 + DT, 60, ch);
      ok = ok && d == JanusMac::DROP && mac.counter() == JanusMac::MAX_COUNTER;
    }
    check(ok, "busy in every slot: dropped when C reaches 8");
  }

  {
    JanusMac mac(CD, true, 3);
    run(mac, 0, 3, idle);
    mac.reserve(4, 20);
    mac.request();
    check(mac.decision() == JanusMac::TRANSMIT, "reservation in the future: idle now");
    mac.transmitted();
    double at = -1;
    run(mac, 3 + DT, 3 + 0.5, idle);
    mac.request();
    JanusMac::Decision d = run(mac, 3.5 + DT, 60, idle, &at);
    check(d == JanusMac::TRANSMIT && at >= 20 + SLOT - 1e-9,
      "received reservation: no TX before its end + guard");
  }

  {
    JanusMac mac(CD, true, 5);
    Channel ch = [](double t) { return t >= 1.0 && t < 1.05; };
    mac.observe(0, false);
    mac.request();
    double at = -1;
    run(mac, DT, 30, ch, &at);
    check(at >= 1.05 + SLOT, "busy during the initial sensing: backoff");
  }

  if(failures == 0) {
    printf("ALL MAC TESTS PASSED\n");
  } else {
    printf("%d MAC TEST(S) FAILED\n", failures);
  }
  return failures == 0 ? 0 : 1;
}
