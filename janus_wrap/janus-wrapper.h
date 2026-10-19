/**
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


/**
 * @file janus_wrapper.h
 * @author Filippo Campagnaro
 * @version 1.0.0
 * @brief .
 */

#ifndef JANUS_WRAPPER_H
#define JANUS_WRAPPER_H


#include <thread>
#include <string>
#include <atomic>
#include <chunk.h>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <janus/janus.h>
#include <janus/defaults.h>

/**
 * .
 */
class JanusWrapper 
{
public:

  /**
  * Channel state for the MAC, sent from the demodulation thread.
  * Times are receiver times in seconds: samples given to demodulate() / fs,
  * so the samples not given (half duplex) do not count.
  */
  struct ChannelEvent {
    double time; /**< time up to which the channel was sensed */
    /** busy window (ANEP-87 §2.2.1), preamble detected or packet received */
    bool busy;
    /** preamble detected (packet being decoded) or packet received */
    bool detected;
    double window_power; /**< libjanus carrier sensing window power */
    double background_power; /**< libjanus carrier sensing background power */
    /** reservation or scheduled repeat received, valid if reserved_to > reserved_from */
    double reserved_from;
    double reserved_to; /**< end of the reserved interval */
  };

  /** Listener of the channel events, called from the demodulation thread */
  typedef std::function<void(const ChannelEvent&)> ChannelListener;

  /**
  * Class constructor.
  */
  JanusWrapper();

  /**
  * Class destructor. It takes care on destroying all liquid-dsp objects.
  */
  ~JanusWrapper();

  /**
  * Function used to modulate a chunk.
  *
  * @param chunk shared pointer of the chunk to be modulated.
  * @return bool with value equal to the result of the function.
  */
  bool modulate(std::shared_ptr<Chunk> chunk);

  /**
  * Function used to recognize an emergency message (ANEP-87 §2.2-3): the
  * user payload starts with "SOS" followed by a blank or nothing. The prefix
  * and the following blanks are removed ("SOS" alone is kept).
  *
  * @param chunk user payload, without SuM MAC header.
  * @return bool true if it is an emergency message.
  */
  static bool emergency(std::shared_ptr<Chunk> chunk);

  /**
  * Function used to modulate an emergency position, class 0 / app type 1
  * (ANEP-87 Annex D 000-01, needs libplugin_000_01.so). The payload is
  * "lat lon [depth [speed [heading [nationality]]]]": degrees, m, kn,
  * degrees, two letters; "-" means not available.
  *
  * @param chunk payload after the "SOS" prefix.
  * @return bool false if the payload is not a position (or out of range).
  */
  bool modulatePosition(std::shared_ptr<Chunk> chunk);

  /**
  * Function used to demodulate a vector struct of samples.
  *
  * @param samples shared pointer of the samples struct to be demodulated.
  * @return bool with value equal to the result of the function.
  */
  bool demodulate(std::shared_ptr<Samples> samples);

  /**
  * Function used to set the pointer for the RX buffer.
  *
  * @return bool with value equal to the result of the function.
  */
  bool setRxBuffer(std::shared_ptr<ConcBuffer> buffer);

  /**
  * Function used to set the carrier frequency
  *
  * @param carr_freq float of the carrier frequency, in Hz.
  * @return bool true if set is successful (false, unchanged, if not positive).
  */
  bool setCarrierFrequency(float carr_freq);

  /**
  * Function used to set the bandwidth.
  *
  * @param bw float of the bandwidth in Hz.
  * @return bool true if set is successful (false, unchanged, unless
  *   0 < bw <= the carrier frequency: set the carrier frequency first).
  */
  bool setBandwidth(float bw);

  /**
  * Function used to set the sampling frequency.
  *
  * @param sampling_frequency_ int of the sampling frequency, in Hz.
  * @return bool false (unchanged) if not positive.
  */
  bool setSamplingFrequency(int sampling_frequency_);

  /**
  * Function used to set the tx_gain for transmission.
  *
  * @param gain float in range [0.0, 1.0]; 0 transmits silence.
  * @return bool false (gain unchanged) if out of range.
  */
  bool setTxGain(float gain);

  /**
  * Function used to set the verbose janus parameter.
  *
  * @param verbose integer in range [0, 2] (from less verbose to more verbose).
  */
  void setVerbose(int verbose);

  /**
  * Function used to enable/disable wake up tones.
  *
  * @param wut integer equal to 1 for enabling wake up tones, 0 for disabling them.
  */
  void setWakeUpTones(int wut);

  /**
  * Function used to enable/disable MAC mechanics.
  *
  * @param wut integer equal to 1 for enabling MAC support, 0 for disabling it.
  */
  void setMacMode(int _macmode);

  /**
  * Function used to set the mobility flag of the baseline packet.
  *
  * @param mob 0 = static, 1 = mobile.
  * @return bool true if set is successful.
  */
  bool setMobility(int mob);

  /**
  * Function used to set the tx/rx capability flag of the baseline packet.
  *
  * @param txrx 0 = tx only, 1 = tx/rx.
  * @return bool true if set is successful.
  */
  bool setTxRx(int txrx);

  /**
  * Function used to set the forwarding capability flag of the baseline packet.
  *
  * @param fwd 0 = no forwarding, 1 = forwarding capable.
  * @return bool true if set is successful.
  */
  bool setForward(int fwd);

  /**
  * Function used to set the class user ID of the baseline packet.
  * A libjanus plugin for (class ID, app type) must exist, see janusSetup().
  *
  * @param cid class user ID in range [0, 255].
  * @return bool true if set is successful.
  */
  bool setClassId(int cid);

  /**
  * Function used to set the application type of the baseline packet.
  *
  * @param at application type in range [0, 63].
  * @return bool true if set is successful.
  */
  bool setAppType(int at);

  /**
  * Function used to set the Station Identifier written in the ADB (class 16).
  *
  * @param sid station ID in range [0, 255].
  * @return bool true if set is successful.
  */
  bool setStationId(int sid);

  /**
  * Function used to set the Parameter Set Identifier written in the ADB (class 16).
  *
  * @param pid parameter set ID in range [0, 4095].
  * @return bool true if set is successful.
  */
  bool setAdbPsetId(int pid);

  /**
  * Function used to set the Destination Identifier written in the ADB
  * (class 11, text messages). Received packets with a different destination
  * (and not broadcast) are dropped.
  *
  * @param did destination ID in range [0, 255], 255 is broadcast.
  * @return bool true if set is successful.
  */
  bool setDestId(int did);

  /**
  * Function used to set the RX filter on class ID and app type.
  *
  * @param cid class ID to accept, -1 to accept any.
  * @param at app type to accept, -1 to accept any.
  * @return bool true if set is successful.
  */
  bool setRxFilter(int cid, int at);

  /**
  * Function used to enable/disable the drop of packets carrying our own
  * Station ID (self interference, loopback of the audio interface).
  *
  * @param enable true to drop own packets.
  */
  void setSelfFilter(bool enable);

  /**
  * Function used to enable/disable the RX metadata prefix
  * (RX_METADATA_SIZE bytes) in front of each received payload.
  *
  * @param enable true to prepend the metadata.
  */
  void setRxMetadata(bool enable);

  /**
  * Function used to set the GO-CFAR preamble detection threshold (receiver
  * choice, not fixed by ANEP-87). Call before janusSetup().
  *
  * @param threshold detection threshold, > 0.
  * @return bool false if out of range.
  */
  bool setDetectionThreshold(double threshold);

  /**
  * Function used to receive the channel state (carrier sensing, received
  * reservations). Call before janusSetup(). The listener must not block.
  *
  * @param listener called from the demodulation thread.
  */
  void setChannelListener(ChannelListener listener);

  /**
  * Function used to get the chip duration of the configured band
  * (same formula as janus pset.c).
  *
  * @return chip duration in seconds.
  */
  double chipDuration() const;

  /**
  * Function used to get the duration of a packet from its detection:
  * preamble, baseline packet and cargo.
  *
  * @param cargo_size cargo size in bytes
  * @return duration in seconds.
  */
  double packetDuration(unsigned int cargo_size) const;

  /**
  * Function used to create all the required janus objects.
  *
  * @return bool false if setup fails, e.g. no plugin for (class ID, app type).
  */
  bool janusSetup();

  /**
  * Size of the RX metadata prefix:
  * station_id u8 | dest_id u8 | pset_id u16 BE | class_id u8 | app_type u8 |
  * flags u8 (bit0 mobility, bit1 tx_rx, bit2 forward).
  */
  static const unsigned int RX_METADATA_SIZE;

  /** Class user ID of the text messages (ANEP-87 Annex D) */
  static const int TEXT_CLASS_ID;

  /** Broadcast Destination Identifier (class 11) */
  static const int BROADCAST_ID;

  /** Class user ID of the emergency messages (ANEP-87 Annex A) */
  static const int EMERGENCY_CLASS_ID;

  /** Application type Emergency Position (ANEP-87 Annex D 000-01) */
  static const int POSITION_APP_TYPE;

  /** Prefix of the emergency messages from the user */
  static const char EMERGENCY_PREFIX[];

private:

  /**
  * function used to check that a libjanus plugin encodes the cargo size
  * for the configured (class ID, app type).
  *
  * @return bool true if the plugin is available.
  */
  bool checkCodec();

  /**
  * function used to modulate a packet ready to be transmitted.
  *
  * @param packet janus packet, freed by this function
  * @param chunk chunk receiving the modulated samples
  * @return bool true if the samples are ready.
  */
  bool modulatePacket(janus_packet_t packet, std::shared_ptr<Chunk> chunk);

  /**
  * function used to fill a janus packet ready to be transmitted: baseline
  * fields, ADB fields, cargo and, for class 11, schedule and reservation time.
  *
  * @param packet pointer to the janus packet
  * @param data cargo data
  * @param len cargo size in bytes
  * @return bool true if the packet is ready.
  */
  bool buildPacket(janus_packet_t packet, const char* data, unsigned int len);

  /**
  * function used to set the schedule flag and the smallest reservation time
  * covering the whole transmission (preamble, baseline packet and cargo).
  *
  * @param packet pointer to the janus packet
  * @param cargo_size cargo size in bytes
  * @return bool true if a suitable reservation time exists.
  */
  bool setReservation(janus_packet_t packet, unsigned int cargo_size);

  /**
  * function used to know if the class carries text messages (class 11): the
  * plugin then builds the cargo (text, padding, CRC16) and needs the schedule.
  *
  * @return bool true for class 11.
  */
  bool isTextClass() const;

  /**
  * function used to compute an upper bound of the TX samples of a packet,
  * including wake up tones (if enabled) and a margin.
  *
  * @param cargo_size cargo size in bytes
  * @return number of samples.
  */
  size_t maxTxSamples(unsigned int cargo_size) const;

  /**
  * function used to initialize the janus packet.
  * 
  * @param packet pointer to the janus packet
  */
  void initPacket(janus_packet_t packet);

  /**
  * function used to push a packet to the rx buffer.
  * 
  * @param rx_chunk chunk received and correctly decoded
  */
  void pushRxChunk(std::shared_ptr<Chunk> rx_chunk);

  /**
  * function used to init the default Janus parameters.
  */
  void initDefaultParams();

  /**
  * function that performs the demodulation in a separated thread.
  */
  void performDemodulation();

  /**
  * function used to send the channel state of a received packet: busy, and
  * its reservation or scheduled repeat if any.
  *
  * @param packet received packet
  * @param time receiver time of the end of the packet
  */
  void notifyPacket(janus_packet_t packet, double time);

  janus_parameters_t params; /**< Janus parameters*/

  janus_simple_tx_t simple_tx; /**< Janus transmitter */
  janus_tx_state_t state_tx;  /**< Janus transmitter state */

  janus_simple_rx_t simple_rx; /**< Janus receiver */
  janus_rx_state_t state_rx;  /**< Janus receiver state */
  janus_carrier_sensing_t carrier_sensing; /**< Janus carrier sensing */
  std::shared_ptr<ConcBuffer> rx_buffer; /**< Shared pointer to rx buffer in PHY */
  
  std::atomic<bool> active; /**< boolean flag to join the thread */
  std::thread demod_thread; /**< Thread that actually computes the demodulation */

  std::mutex mutex_demod; /**< mutex for demodulating */
  /** condition variable to wait for data to be demodulated */
  std::condition_variable cv_demod; 

  int demod_data_available; /**< boolean data samples inserted*/

  unsigned long long samples_in; /**< samples given to the demodulator */

  ChannelListener channel_listener; /**< receiver of the channel state */

  float carrier_freq; /**< Carrier frequency for framegenerator, in Hz */

  unsigned int samples_per_symbol; /**< Number of samples for a single symbol */

  float sampling_frequency; /**< Sampling frequency, in Hz */

  float bandwidth; /**< Bandwidth in Hz */

  static const unsigned int JANUS_RED_SIZE; /**< constant equal to JANUS DEFAULT_DRV_IN_SIZE */
  static const unsigned int MAX_CARGO_SIZE; /**< max JANUS cargo in bytes (plugins) */
  static const double TX_MARGIN; /**< extra TX time in s (filters, padding) */

  int mac_mode; /**< MAC mode, 0 is disabled, 1 enabled */

  int mobility; /**< mobility flag, 0 static, 1 mobile */
  int tx_rx; /**< tx/rx flag, 0 tx only, 1 tx/rx */
  int forward; /**< forwarding capability flag */
  int class_id; /**< class user ID */
  int app_type; /**< application type */
  int station_id; /**< Station Identifier in the ADB */
  int adb_pset_id; /**< Parameter Set Identifier in the ADB */
  int dest_id; /**< Destination Identifier in the ADB (class 11) */
  std::string tx_fifo_args; /**< TX fifo size, sized for the longest packet */
  int filter_class_id; /**< RX class ID filter, -1 accepts any */
  int filter_app_type; /**< RX app type filter, -1 accepts any */
  bool self_filter; /**< drop packets carrying our own Station ID */
  bool rx_metadata; /**< prepend RX metadata to received payloads */
};

#endif
