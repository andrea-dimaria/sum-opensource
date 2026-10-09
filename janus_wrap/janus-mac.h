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

#ifndef JANUS_MAC_H
#define JANUS_MAC_H

#include <random>
#include <vector>
#include <utility>

/**
 * JANUS default MAC (ANEP-87 §2.2): CSMA with Global Awareness (in-band
 * energy detector, busy if a 16 Cd window is 3 dB over the background) and
 * Binary Exponential Backoff.
 *
 * Pure state machine: no audio, no threads, no clock. Time is the receiver
 * time in seconds (samples given to the demodulator / fs), so the samples
 * discarded while transmitting (half duplex) do not count as sensed.
 * Not thread safe: the caller serializes the calls.
 *
 * Rules:
 * - sense the channel for at least 352 Cd before a transmission (§2.2.1-2);
 *   the sensing restarts after each own transmission;
 * - idle (nothing busy in the last 176 Cd): transmit immediately (§2.2.1-5);
 * - otherwise wait until not busy, then slots of 176 Cd: transmit at the
 *   start of a slot with P = 1/(1+2^(C-2)) for C < 4, 1/5 for 4 <= C < 8;
 *   C counts the slots with a busy window, the packet is dropped at C = 8
 *   (§2.2.2);
 * - never start less than 176 Cd after the last busy window (§2.2.3-2);
 * - band overlapping the underwater telephone band: never transmit in the
 *   slot after a busy slot (§2.2.2-2);
 * - a received reservation makes the channel busy for its interval (§2.2-2);
 * - emergency transmissions do not use the MAC (§2.2-3), the caller skips it.
 */
class JanusMac
{
public:

  /** Decision for the pending packet */
  enum Decision {
    NONE, /**< no packet pending */
    WAIT, /**< keep sensing */
    TRANSMIT, /**< transmit now */
    DROP /**< C reached 8: drop the packet */
  };

  /**
  * Class constructor.
  *
  * @param chip_duration chip duration Cd in seconds.
  * @param ut_band true if the band overlaps the underwater telephone band.
  * @param seed seed of the random generator of the backoff.
  */
  JanusMac(double chip_duration, bool ut_band, unsigned int seed);

  /**
  * Function used to give the channel state sensed up to time t.
  *
  * @param t receiver time in seconds, not decreasing.
  * @param busy true if a busy window (or a packet) was detected since the
  *        previous call.
  */
  void observe(double t, bool busy);

  /**
  * Function used to mark the channel busy in [from, to] (reservation or
  * scheduled repeat received from another node).
  *
  * @param from start, receiver time in seconds.
  * @param to end, receiver time in seconds.
  */
  void reserve(double from, double to);

  /**
  * Function used to start the access for a new packet (non emergency).
  */
  void request();

  /**
  * Function used to know what to do with the pending packet.
  *
  * @return the decision at the last observed time.
  */
  Decision decision() const;

  /**
  * Function used to tell that the pending packet was transmitted, or that an
  * emergency packet was: the sensing restarts, C = 1.
  */
  void transmitted();

  /**
  * Function used to forget the pending packet (after DROP).
  */
  void cancel();

  /** @return backoff counter C. */
  int counter() const { return c; }

  /** @return last observed time in seconds. */
  double now() const { return t_now; }

  /** @return time sensed since the last restart, in seconds. */
  double sensed() const;

  /** Sensing time before a transmission, in Cd */
  static const double SENSE_CHIPS;
  /** Slot and guard time, in Cd */
  static const double SLOT_CHIPS;
  /** C value at which the packet is dropped */
  static const int MAX_COUNTER;

  /**
  * Function used to know if a band overlaps the underwater telephone band
  * (STANAG 1475).
  *
  * @param center_freq center frequency in Hz.
  * @param bandwidth bandwidth in Hz.
  * @return true if the bands overlap.
  */
  static bool overlapsUtBand(double center_freq, double bandwidth);

  /**
  * Function used to get the transmission probability of the backoff.
  *
  * @param counter backoff counter C, in [1, 7].
  * @return probability of transmitting in the next slot.
  */
  static double probability(int counter);

private:

  /** Access state of the pending packet */
  enum State { IDLE, SENSING, WAIT_CLEAR, BACKOFF, READY, DROPPED };

  /**
  * function used to advance the state machine after an observation.
  *
  * @param busy_now true if the last observation was busy.
  */
  void step(bool busy_now);

  /**
  * function used to evaluate the start of a backoff slot.
  *
  * @param busy_now true if the last observation was busy.
  */
  void slotStart(bool busy_now);

  /**
  * function used to know if [from, to] overlaps a reservation.
  */
  bool reserved(double from, double to) const;

  double slot; /**< slot and guard time, in s */
  double sense; /**< sensing time before a transmission, in s */
  bool ut_band; /**< band overlapping the underwater telephone band */
  std::mt19937 rng; /**< random generator of the backoff */

  State state; /**< access state */
  bool started; /**< at least one observation */
  double t_now; /**< last observed time */
  double t_sense; /**< start of the current sensing */
  double t_busy; /**< last busy time */
  bool ever_busy; /**< t_busy is valid */
  int c; /**< backoff counter C */
  double t_slot; /**< start of the current slot */
  bool slot_busy; /**< busy window in the current slot */
  bool prev_slot_busy; /**< busy window in the previous slot */
  std::vector<std::pair<double, double>> reservations; /**< busy intervals */
};

#endif
