/*
 * [2024] SubSeaPulse SRL
 * All Rights Reserved.
 */
// This is free software: you can redistribute it and/or modify it        *
// under the terms of the GNU General Public License version 3 as         *
// published by the Free Software Foundation.                             *
//                                                                        *
// This program is distributed in the hope that it will be useful, but    *
// WITHOUT ANY WARRANTY; without even the implied warranty of FITNESS     *
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for       *
// more details.                                                          *
//                                                                        *
// You should have received a copy of the GNU General Public License      *
// along with this program. If not, see <http://www.gnu.org/licenses/>.   *

#include "janus-mac.h"

#include <algorithm>
#include <cmath>

const double JanusMac::SENSE_CHIPS = 352;
const double JanusMac::SLOT_CHIPS = 176;
const int JanusMac::MAX_COUNTER = 8;

// STANAG 1475 underwater telephone band (SSB voice above the 8.0875 kHz
// carrier, about 8-11 kHz)
static const double UT_BAND_LOW = 8000.0;
static const double UT_BAND_HIGH = 11000.0;

JanusMac::JanusMac(double chip_duration, bool ut_band, unsigned int seed)
:
  slot(SLOT_CHIPS * chip_duration),
  sense(SENSE_CHIPS * chip_duration),
  ut_band(ut_band),
  rng(seed),
  state(IDLE),
  started(false),
  t_now(0),
  t_sense(0),
  t_busy(0),
  ever_busy(false),
  c(1),
  t_slot(0),
  slot_busy(false),
  prev_slot_busy(false),
  reservations()
{
}

bool
JanusMac::overlapsUtBand(double center_freq, double bandwidth)
{
  return center_freq - bandwidth / 2 < UT_BAND_HIGH
    && center_freq + bandwidth / 2 > UT_BAND_LOW;
}

double
JanusMac::probability(int counter)
{
  if(counter < 4) {
    return 1.0 / (1.0 + std::pow(2.0, counter - 2));
  }
  return 1.0 / 5.0;
}

double
JanusMac::sensed() const
{
  return started ? t_now - t_sense : 0;
}

bool
JanusMac::reserved(double from, double to) const
{
  for(const auto& r : reservations) {
    if(r.first <= to && r.second >= from) {
      return true;
    }
  }
  return false;
}

void
JanusMac::observe(double t, bool busy)
{
  if(!started) {
    started = true;
    t_now = t;
    t_sense = t;
  }
  if(t < t_now) {
    t = t_now;
  }
  busy = busy || reserved(t_now, t);
  t_now = t;
  if(busy) {
    t_busy = t;
    ever_busy = true;
  }
  reservations.erase(std::remove_if(reservations.begin(), reservations.end(),
    [t](const std::pair<double, double>& r) { return r.second < t; }),
    reservations.end());
  step(busy);
}

void
JanusMac::reserve(double from, double to)
{
  if(to > from) {
    reservations.emplace_back(from, to);
  }
}

void
JanusMac::request()
{
  state = SENSING;
  c = 1;
  step(ever_busy && t_busy == t_now);
}

JanusMac::Decision
JanusMac::decision() const
{
  switch(state) {
    case IDLE: return NONE;
    case READY: return TRANSMIT;
    case DROPPED: return DROP;
    default: return WAIT;
  }
}

void
JanusMac::transmitted()
{
  // deaf while transmitting: the background must be sensed again
  t_sense = t_now;
  state = IDLE;
  c = 1;
}

void
JanusMac::cancel()
{
  state = IDLE;
  c = 1;
}

void
JanusMac::step(bool busy_now)
{
  switch(state) {
    case SENSING:
      if(t_now - t_sense < sense) {
        return;
      }
      // idle channel: transmit immediately; any recent activity means
      // contention, resolved by the backoff instead of the guard time alone
      if(!ever_busy || t_now - t_busy >= slot) {
        state = READY;
        return;
      }
      state = WAIT_CLEAR;
      // fall through
    case WAIT_CLEAR:
      if(busy_now) {
        return;
      }
      // the busy period counts as the slot before the first one
      state = BACKOFF;
      t_slot = t_now;
      slot_busy = false;
      prev_slot_busy = true;
      slotStart(busy_now);
      return;
    case BACKOFF:
      if(busy_now) {
        slot_busy = true;
      }
      while(state == BACKOFF && t_now >= t_slot + slot) {
        if(slot_busy && ++c >= MAX_COUNTER) {
          state = DROPPED;
          return;
        }
        prev_slot_busy = slot_busy;
        slot_busy = busy_now; // the observation ends in the new slot
        t_slot += slot;
        slotStart(busy_now);
      }
      return;
    default:
      return;
  }
}

void
JanusMac::slotStart(bool busy_now)
{
  if(busy_now || t_now - t_busy < slot || (ut_band && prev_slot_busy)) {
    return;
  }
  if(std::uniform_real_distribution<double>(0.0, 1.0)(rng) < probability(c)) {
    state = READY;
  }
}
