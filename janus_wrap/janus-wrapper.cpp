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

#include <janus-wrapper.h>
#include <string.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <janus/dump.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include <cctype>

#define STATION_ID_LABEL "StationIdentifier"
#define PSET_ID_LABEL "ParameterSetIdentifier"
#define DEST_ID_LABEL "DestinationIdentifier"
#define CODING_LABEL "Coding"
#define ENCRYPTION_LABEL "Encryption"
#define PAYLOAD_SIZE_LABEL "Payload Size"
#define PAYLOAD_LABEL "Payload"
// class 0 / app type 1, Emergency Position (plugin_000_01)
static const char* const POSITION_LABELS[] = {"Latitude", "Longitude",
  "Depth", "Speed", "Heading", "Nationality"};
static const unsigned int POSITION_FIELDS = 6;

const unsigned int JanusWrapper::JANUS_RED_SIZE = 1024;
const unsigned int JanusWrapper::MAX_CARGO_SIZE = 480;
const double JanusWrapper::TX_MARGIN = 0.5;
const unsigned int JanusWrapper::RX_METADATA_SIZE = 7;
const int JanusWrapper::TEXT_CLASS_ID = 11;
const int JanusWrapper::BROADCAST_ID = 255;
const int JanusWrapper::EMERGENCY_CLASS_ID = 0;
const int JanusWrapper::POSITION_APP_TYPE = 1;
const char JanusWrapper::EMERGENCY_PREFIX[] = "SOS";

JanusWrapper::JanusWrapper()
:
  params(nullptr),
  simple_tx(nullptr),
  state_tx(nullptr),
  simple_rx(nullptr),
  state_rx(nullptr),
  carrier_sensing(nullptr),
  rx_buffer(nullptr),
  active(false),
  demod_thread(),
  mutex_demod(),
  cv_demod(),
  demod_data_available(0),
  samples_in(0),
  channel_listener(),
  mac_mode(0),
  mobility(0),
  tx_rx(1),
  forward(0),
  class_id(16),
  app_type(0),
  station_id(0),
  adb_pset_id(0),
  dest_id(255),
  filter_class_id(-1),
  filter_app_type(-1),
  self_filter(false),
  rx_metadata(false)
{
  params = janus_parameters_new();
  initDefaultParams();
}


JanusWrapper::~JanusWrapper()
{
  active = false;
  cv_demod.notify_one();
  if(demod_thread.joinable())
    demod_thread.join();
  janus_parameters_free(params);
  if(simple_tx)
    janus_simple_tx_free(simple_tx);
  if(simple_rx)
    janus_simple_rx_free(simple_rx);
  if(state_tx)
    janus_tx_state_free(state_tx);
  if(state_rx)
    janus_rx_state_free(state_rx);
  if(carrier_sensing)
    janus_carrier_sensing_free(carrier_sensing);
}

bool 
JanusWrapper::janusSetup() 
{
  if(!checkCodec()) {
    return false;
  }
  // the TX fifo must hold the whole longest packet: when full, janus drops
  // the remaining samples silently
  tx_fifo_args = std::to_string(maxTxSamples(MAX_CARGO_SIZE) 
    * sizeof(int16_t) * params->stream_channel_count);
  const char* rx_fifo_args = params->stream_driver_args;
  params->stream_driver_args = tx_fifo_args.c_str();
  simple_tx = janus_simple_tx_new(params);
  params->stream_driver_args = rx_fifo_args;
  simple_rx = janus_simple_rx_new(params);
  if(params->verbose > 0) {
    std::cout << "JANUS TX fifo: " << tx_fifo_args << " bytes, max packet "
      << packetDuration(MAX_CARGO_SIZE) << " s" << std::endl;
  }
  state_tx = janus_tx_state_new((params->verbose > 1));
  state_rx = janus_rx_state_new(params);
  carrier_sensing = janus_carrier_sensing_new(janus_simple_rx_get_rx(simple_rx));
  active = true;
  demod_thread = std::thread(&JanusWrapper::performDemodulation,this);
  return true;
}

void 
JanusWrapper::initDefaultParams()
{
  params->verbose = 0;
  params->pset_id = 0;
  params->pset_center_freq = 11520;
  params->pset_bandwidth = 4160;
  params->stream_channel = 0;
  params->stream_channel_count = 1;
  params->stream_passband = 1;
  params->wut = 0;
  //params->stream_driver = "alsa";
  params->stream_driver = "fifo";
  //params->stream_driver_args = "default"; 
  params->stream_driver_args = "2000000"; // RX fifo, TX sized in janusSetup
  params->stream_fs = 44100;
  params->stream_format = "S16";
  params->stream_passband = 1;
  params->stream_amp = JANUS_REAL_CONST(0.95);
  params->stream_mul = 1;
  params->doppler_correction = 1;
  params->doppler_max_speed = JANUS_REAL_CONST(5.0);
  params->compute_channel_spectrogram = 1;
  // libjanus uses 2.5; on the modems that gave a false preamble detection
  // every ~30 s on sea noise alone, each one making the receiver deaf for a
  // packet length, so back-to-back packets were lost. 3.0 removed them.
  params->detection_threshold = JANUS_REAL_CONST(3.0);
  params->colored_bit_prob = 0;
  params->cbp_high2medium  = JANUS_REAL_CONST(0.2);
  params->cbp_medium2low   = JANUS_REAL_CONST(0.35);
}
void 
JanusWrapper::initPacket(janus_packet_t packet)
{
  janus_packet_set_mobility(packet, mobility);
  janus_packet_set_tx_rx(packet, tx_rx);
  janus_packet_set_forward(packet, forward);
  janus_packet_set_class_id(packet, class_id);
  janus_packet_set_app_type(packet, app_type);
  janus_packet_set_validity(packet, 1); 
}

bool
JanusWrapper::isTextClass() const
{
  return class_id == TEXT_CLASS_ID;
}

double
JanusWrapper::chipDuration() const
{
  // same chip duration as janus pset.c
  janus_real_t chip_frq = JANUS_ROUND(params->pset_bandwidth
    / (janus_real_t)(JANUS_CHIP_FRQ_COUNT * JANUS_ALPHABET_SIZE));
  return JANUS_POW(JANUS_REAL_CONST(2.0), params->chip_len_exp) / chip_frq;
}

double
JanusWrapper::packetDuration(unsigned int cargo_size) const
{
  unsigned chips = JANUS_PREAMBLE_CHIP_COUNT
    + (JANUS_MIN_PKT_SIZE_BITS + JANUS_CONV_MEM_SIZE_BITS) * JANUS_ALPHABET_SIZE;
  if(cargo_size > 0) {
    chips += (cargo_size * 8 + JANUS_CONV_MEM_SIZE_BITS) * JANUS_ALPHABET_SIZE;
  }
  return chips * chipDuration();
}

size_t
JanusWrapper::maxTxSamples(unsigned int cargo_size) const
{
  double duration = packetDuration(cargo_size) + TX_MARGIN;
  if(params->wut) {
    // three wake up tones and a silence gap
    duration += 3 * JANUS_WAKE_UP_TONE_CHIP_COUNT * chipDuration() 
      + JANUS_WUT_GAP;
  }
  return (size_t)(duration * params->stream_fs) + 1;
}

bool
JanusWrapper::setReservation(janus_packet_t packet, unsigned int cargo_size)
{
  // libjanus RX counts the reservation from the packet detection
  double duration = packetDuration(cargo_size);
  double desired = duration;
  double found = 0.0;
  double next = 0.0;
  for(int i = 0; i < 4; ++i) {
    switch(janus_packet_set_tx_reservation_time(packet, desired, &found, &next)) {
      case JANUS_PACKET_EXACT_TIME:
        return true;
      case JANUS_PACKET_APPROXIMATED_TIME:
        if(found >= duration) {
          return true;
        }
        desired = found * 1.1; // next table value, never shorter than needed
        break;
      case JANUS_PACKET_BETWEEN_TWO_VALUES:
        desired = next;
        break;
      case JANUS_PACKET_ERROR_MIN:
        desired = found;
        break;
      default:
        return false;
    }
  }
  return false;
}

bool
JanusWrapper::buildPacket(janus_packet_t packet, const char* data, unsigned int len)
{
  initPacket(packet);
  janus_app_fields_t fields = janus_app_fields_new();
  // each plugin encodes only the fields it knows
  janus_app_fields_add_field(fields, STATION_ID_LABEL,
    std::to_string(station_id).c_str());
  janus_app_fields_add_field(fields, PSET_ID_LABEL,
    std::to_string(adb_pset_id).c_str());
  janus_app_fields_add_field(fields, DEST_ID_LABEL,
    std::to_string(dest_id).c_str());
  janus_app_fields_add_field(fields, CODING_LABEL, "0"); // 8 bit ASCII
  janus_app_fields_add_field(fields, ENCRYPTION_LABEL, "0"); // no encryption
  bool ok = true;
  if(isTextClass()) {
    // the plugin builds text, padding and CRC16 from zero terminated text
    if(len == 0 || memchr(data, 0, len) != nullptr) {
      ok = false;
    } else {
      std::string text(data, len);
      janus_app_fields_add_field(fields, PAYLOAD_SIZE_LABEL,
        std::to_string(len).c_str());
      janus_app_fields_add_field(fields, PAYLOAD_LABEL, text.c_str());
      janus_packet_set_application_data_fields(packet, fields);
      ok = janus_packet_encode_cargo(packet) == 0 
        && setReservation(packet, janus_packet_get_desired_cargo_size(packet));
    }
  } else {
    janus_packet_set_application_data_fields(packet, fields);
    ok = janus_packet_set_cargo(packet, (janus_uint8_t*)data, len) 
      == JANUS_ERROR_NONE;
  }
  janus_app_fields_free(fields);
  return ok && janus_packet_encode_application_data(packet) == 0;
}

bool
JanusWrapper::checkCodec()
{
  // the cargo size is encoded only by the plugin of (class ID, app type):
  // without it the cargo is never transmitted
  janus_packet_t packet = janus_packet_new(params->verbose);
  const char probe[] = "x";
  bool ok = buildPacket(packet, probe, 1)
    && janus_packet_get_cargo_size(packet) > 0;
  janus_packet_free(packet);
  if(!ok) {
    char plugin[32];
    snprintf(plugin, sizeof(plugin), "libplugin_%03d_%02d.so", class_id, app_type);
    std::cerr << "ERROR: no libjanus plugin for class ID " << class_id
      << " app type " << app_type << " (" << plugin 
      << ") or invalid ADB fields" << std::endl;
  }
  return ok;
}
bool
JanusWrapper::setCarrierFrequency(float carr_freq)
{
  if(carr_freq != 0.0) {
    params->pset_center_freq = carr_freq;
    return true;
  }
  return false;
}

bool
JanusWrapper::setBandwidth(float bw)
{
  if(bw > 0.0 && bw <= params->pset_center_freq) 
  {
    params->pset_bandwidth = bw;
    return true;
  }
  return false;
}

void 
JanusWrapper::setVerbose(int verbose)
{
  params->verbose = verbose;
}

void 
JanusWrapper::setWakeUpTones(int wut)
{
  params->wut = wut;
}

void
JanusWrapper::setMacMode(int _mac_mode){
  mac_mode = _mac_mode;
}

bool
JanusWrapper::setMobility(int mob)
{
  if(mob == 0 || mob == 1) {
    mobility = mob;
    return true;
  }
  return false;
}

bool
JanusWrapper::setTxRx(int txrx)
{
  if(txrx == 0 || txrx == 1) {
    tx_rx = txrx;
    return true;
  }
  return false;
}

bool
JanusWrapper::setForward(int fwd)
{
  if(fwd == 0 || fwd == 1) {
    forward = fwd;
    return true;
  }
  return false;
}

bool
JanusWrapper::setClassId(int cid)
{
  if(cid >= 0 && cid <= 255) {
    class_id = cid;
    return true;
  }
  return false;
}

bool
JanusWrapper::setAppType(int at)
{
  if(at >= 0 && at <= 63) {
    app_type = at;
    return true;
  }
  return false;
}

bool
JanusWrapper::setStationId(int sid)
{
  if(sid >= 0 && sid <= 255) {
    station_id = sid;
    return true;
  }
  return false;
}

bool
JanusWrapper::setAdbPsetId(int pid)
{
  if(pid >= 0 && pid <= 4095) {
    adb_pset_id = pid;
    return true;
  }
  return false;
}

bool
JanusWrapper::setDestId(int did)
{
  if(did >= 0 && did <= 255) {
    dest_id = did;
    return true;
  }
  return false;
}

bool
JanusWrapper::setRxFilter(int cid, int at)
{
  if(cid < -1 || cid > 255 || at < -1 || at > 63) {
    return false;
  }
  filter_class_id = cid;
  filter_app_type = at;
  return true;
}

void
JanusWrapper::setSelfFilter(bool enable)
{
  self_filter = enable;
}

void
JanusWrapper::setRxMetadata(bool enable)
{
  rx_metadata = enable;
}

void
JanusWrapper::setChannelListener(ChannelListener listener)
{
  channel_listener = listener;
}

bool
JanusWrapper::setDetectionThreshold(double threshold)
{
  if(threshold <= 0) {
    return false;
  }
  params->detection_threshold = threshold;
  return true;
}

void
JanusWrapper::setSamplingFrequency(int sampling_frequency_)
{
  if(sampling_frequency_ != 0.0) {
    params->stream_fs = sampling_frequency_;
  }
}


bool
JanusWrapper::setRxBuffer(std::shared_ptr<ConcBuffer> buffer)
{
  if(buffer == nullptr) {
    return false;
  }

  rx_buffer = buffer;
  return true;
}

bool
JanusWrapper::setTxGain(float gain)
{
  // gain 0 mutes: it used to be ignored, leaving libjanus's default amplitude 0.95
  if(gain >= 0.0f && gain <= 1.0f) {
    params->stream_amp = JANUS_REAL_CONST(gain);
    return true;
  }
  return false;
}

void JanusWrapper::pushRxChunk(std::shared_ptr<Chunk> rx_chunk)
{
  rx_buffer->push(rx_chunk);
}

bool 
JanusWrapper::modulate(std::shared_ptr<Chunk> chunk)
{
  if(chunk->getSize() == 0) {
    return false;
  } 
  janus_packet_t packet = janus_packet_new(params->verbose);
  char data[chunk->getHeaderSize() + chunk->getSize()] = {0};
  memcpy(data,chunk->getHeader(), chunk->getHeaderSize());
  memcpy(data + chunk->getHeaderSize(),chunk->data(), chunk->getSize());

  if(!buildPacket(packet, data, chunk->getHeaderSize() + chunk->getSize())) {
    janus_packet_free(packet);
    return false;
  }
  return modulatePacket(packet, chunk);
}

bool
JanusWrapper::emergency(std::shared_ptr<Chunk> chunk)
{
  const size_t len = sizeof(EMERGENCY_PREFIX) - 1;
  const char* data = chunk->data();
  size_t size = chunk->getSize();
  if(size < len || memcmp(data, EMERGENCY_PREFIX, len) != 0
    || (size > len && !isspace((unsigned char)data[len]))) {
    return false;
  }
  size_t skip = len;
  while(skip < size && isspace((unsigned char)data[skip])) {
    ++skip;
  }
  // "SOS" alone is sent as it is: an empty payload cannot be sent
  if(skip < size) {
    memmove(chunk->data(), data + skip, size - skip);
    chunk->setSize(size - skip);
  }
  return true;
}

bool
JanusWrapper::modulatePosition(std::shared_ptr<Chunk> chunk)
{
  // lat lon [depth [speed [heading [nationality]]]], "-" = not available
  std::istringstream in(std::string(chunk->data(), chunk->getSize()));
  std::vector<std::string> tokens;
  std::string token;
  while(in >> token) {
    tokens.push_back(token);
  }
  if(tokens.size() < 2 || tokens.size() > POSITION_FIELDS) {
    return false;
  }
  for(size_t i = 0; i < tokens.size(); ++i) {
    char* end = nullptr;
    bool number = (strtod(tokens[i].c_str(), &end), *end == 0);
    bool optional = i >= 2 && tokens[i] == "-";
    bool nationality = i == 5 && tokens[i].size() == 2
      && isalpha((unsigned char)tokens[i][0]) && isalpha((unsigned char)tokens[i][1]);
    if(!(optional || (i < 5 && number) || nationality)) {
      return false;
    }
  }
  janus_packet_t packet = janus_packet_new(params->verbose);
  initPacket(packet);
  janus_packet_set_class_id(packet, EMERGENCY_CLASS_ID);
  janus_packet_set_app_type(packet, POSITION_APP_TYPE);
  janus_app_fields_t fields = janus_app_fields_new();
  janus_app_fields_add_field(fields, STATION_ID_LABEL,
    std::to_string(station_id).c_str());
  janus_app_fields_add_field(fields, DEST_ID_LABEL,
    std::to_string(dest_id).c_str());
  for(size_t i = 0; i < tokens.size(); ++i) {
    if(tokens[i] != "-") {
      janus_app_fields_add_field(fields, POSITION_LABELS[i], tokens[i].c_str());
    }
  }
  janus_packet_set_application_data_fields(packet, fields);
  // the plugin builds the 14-byte cargo; Annex D 000-01 needs a reservation
  // covering it
  bool ok = janus_packet_encode_cargo(packet) == 0
    && setReservation(packet, janus_packet_get_desired_cargo_size(packet))
    && janus_packet_encode_application_data(packet) == 0;
  janus_app_fields_free(fields);
  if(!ok) {
    janus_packet_free(packet);
    return false;
  }
  return modulatePacket(packet, chunk);
}

bool
JanusWrapper::modulatePacket(janus_packet_t packet, std::shared_ptr<Chunk> chunk)
{
  janus_packet_set_validity(packet, 2);
  if(janus_simple_tx_execute(simple_tx, packet, state_tx) < 0) {
    janus_packet_free(packet);
    return false;
  }
  
  janus_tx_state_dump(state_tx);
  janus_packet_dump(packet);

  janus_ostream_t ostream = janus_simple_tx_get_ostream(simple_tx);
  unsigned int cargo_size = janus_packet_get_cargo_size(packet);
  size_t capacity = maxTxSamples(cargo_size) * params->stream_channel_count;
  std::shared_ptr<Samples> samples = std::make_shared<Samples>(capacity);
  size_t red_total = 0;
  int red_size = 0;
  // drain the TX fifo, growing the buffer if needed
  do {
    if(red_total == samples->samples.size()) {
      samples->samples.resize(2 * samples->samples.size());
    }
    red_size = ostream->read(ostream, samples->samples.data() + red_total,
      (samples->samples.size() - red_total) / params->stream_channel_count);
    if(red_size > 0) {
      red_total += red_size * params->stream_channel_count;
    }
  } while(red_size > 0);
  // a shorter signal means the fifo dropped samples: never transmit it
  size_t min_samples = (size_t)(packetDuration(cargo_size) 
    * params->stream_fs) * params->stream_channel_count;
  if(red_size < 0 || red_total < min_samples) {
    std::cerr << "JANUS TX: " << red_total << " samples, expected at least "
      << min_samples << std::endl;
    janus_packet_free(packet);
    return false;
  }
  samples->samples.resize(red_total);
  samples->sample_size = red_total;
  chunk->setModulatedSamples(samples);
  janus_packet_free(packet);
  return true;

}
bool 
JanusWrapper::demodulate(std::shared_ptr<Samples> samples)
{
  janus_istream_t istream = janus_simple_rx_get_istream(simple_rx);
  std::unique_lock<std::mutex> lk(mutex_demod);
  istream->write(istream,samples->samples.data(),samples->sample_size);
  demod_data_available += samples->sample_size;
  samples_in += samples->sample_size / params->stream_channel_count;
  if(demod_data_available >= 10) {
    cv_demod.notify_one();
  }
  lk.unlock();  
  return true;
}

void
JanusWrapper::notifyPacket(janus_packet_t packet, double time)
{
  if(!channel_listener) {
    return;
  }
  ChannelEvent ev = {time, true, true, 0, 0, 0, 0};
  janus_uint8_t flag = 2;
  double interval = janus_packet_get_validity(packet)
    ? janus_packet_get_tx_interval(packet, &flag) : 0.0;
  if(interval > 0) {
    // both intervals count from the packet detection (ANEP-87 Annex B)
    double duration = packetDuration(janus_packet_get_cargo_size(packet));
    double start = time - duration;
    if(flag == 0) { // RPT flag 0: reservation time (packet.c, not exported)
      ev.reserved_from = start;
      ev.reserved_to = start + interval;
    } else {
      // scheduled repeat: the same packet again after the interval
      ev.reserved_from = start + interval;
      ev.reserved_to = start + interval + duration;
    }
  }
  channel_listener(ev);
}

void JanusWrapper::performDemodulation()
{
  unsigned queried_detection_time = 0;
  janus_packet_t packet_rx = janus_packet_new(params->verbose);
  janus_real_t time;
  double last_cs_time = -1;
  initPacket(packet_rx);

  while(active.load()) {
    std::unique_lock<std::mutex> lck{mutex_demod};
    cv_demod.wait(lck,[&](){
      return demod_data_available >= JanusWrapper::JANUS_RED_SIZE || !(active.load());
    }); 
    int rv = janus_simple_rx_execute(simple_rx, packet_rx, state_rx);
    
    if (rv < 0) // error in reading
    {
      lck.unlock();
      if (rv == JANUS_ERROR_OVERRUN)
      {
        // error happens
      }
    }
    else if (rv > 0)
    {
      demod_data_available -= JanusWrapper::JANUS_RED_SIZE;
      if(demod_data_available < 0) {
        demod_data_available = 0;
      }
      double rx_time = (samples_in - demod_data_available
        / params->stream_channel_count) / (double)params->stream_fs;
      lck.unlock();
      notifyPacket(packet_rx, rx_time);

      if (janus_packet_get_validity(packet_rx) &&
            janus_packet_get_cargo_error(packet_rx) == 0)
      {
        unsigned int _payload_len = janus_packet_get_cargo_size(packet_rx);
        int rx_class_id = janus_packet_get_class_id(packet_rx);
        int rx_app_type = janus_packet_get_app_type(packet_rx);
        int rx_station_id = -1;
        int rx_pset_id = -1;
        int rx_dest_id = -1;
        std::map<std::string, std::string> position;
        janus_app_fields_t fields = janus_app_fields_new();
        janus_packet_get_application_data_fields(packet_rx, fields);
        for(int i = 0; i < fields->field_count; ++i) {
          if(strcmp(fields->fields[i].name, STATION_ID_LABEL) == 0) {
            rx_station_id = atoi(fields->fields[i].value);
          } else if(std::find(POSITION_LABELS, POSITION_LABELS + POSITION_FIELDS,
              std::string(fields->fields[i].name)) != POSITION_LABELS + POSITION_FIELDS) {
            position[fields->fields[i].name] = fields->fields[i].value;
          } else if(strcmp(fields->fields[i].name, PSET_ID_LABEL) == 0) {
            rx_pset_id = atoi(fields->fields[i].value);
          } else if(strcmp(fields->fields[i].name, DEST_ID_LABEL) == 0) {
            rx_dest_id = atoi(fields->fields[i].value);
          } else if(strcmp(fields->fields[i].name, PAYLOAD_SIZE_LABEL) == 0) {
            // actual payload, without the plugin padding and CRC
            unsigned int size = atoi(fields->fields[i].value);
            if(size < _payload_len) {
              _payload_len = size;
            }
          }
        }
        janus_app_fields_free(fields);

        // emergency positions are always delivered, whatever the filters
        bool emergency_rx = rx_class_id == EMERGENCY_CLASS_ID
          && rx_app_type == POSITION_APP_TYPE && position.count("Latitude");
        std::string sos;
        if(emergency_rx) {
          // same syntax as the TX: SOS lat lon depth speed heading nationality
          sos = EMERGENCY_PREFIX;
          for(unsigned int i = 0; i < POSITION_FIELDS; ++i) {
            auto it = position.find(POSITION_LABELS[i]);
            sos += " " + (it == position.end() ? std::string("-") : it->second);
          }
          _payload_len = sos.size();
        }
        bool accepted = 
          (emergency_rx || filter_class_id < 0 || filter_class_id == rx_class_id) &&
          (emergency_rx || filter_app_type < 0 || filter_app_type == rx_app_type) &&
          !(self_filter && rx_station_id == station_id) &&
          (rx_dest_id < 0 || rx_dest_id == BROADCAST_ID 
            || rx_dest_id == station_id) &&
          _payload_len > 0;
        if(params->verbose > 0) {
          std::cout << "JANUS RX: class_id = " << rx_class_id
            << " app_type = " << rx_app_type
            << " station_id = " << rx_station_id
            << " dest_id = " << rx_dest_id
            << " pset_id = " << rx_pset_id
            << " cargo_size = " << _payload_len
            << (accepted ? "" : " DROPPED") << std::endl;
        }

        // optional metadata prefix in front of the payload sent to the user
        unsigned int md_size = rx_metadata ? RX_METADATA_SIZE : 0;
        char md[RX_METADATA_SIZE] = {0};
        if(rx_metadata) {
          janus_uint16_t pid = rx_pset_id < 0 ? 0 : rx_pset_id;
          md[0] = (char)(rx_station_id < 0 ? 0 : rx_station_id);
          md[1] = (char)(rx_dest_id < 0 ? BROADCAST_ID : rx_dest_id);
          md[2] = (char)(pid >> 8);
          md[3] = (char)(pid & 0xFF);
          md[4] = (char)rx_class_id;
          md[5] = (char)rx_app_type;
          md[6] = (char)((janus_packet_get_mobility(packet_rx) & 1)
            | ((janus_packet_get_tx_rx(packet_rx) & 1) << 1)
            | ((janus_packet_get_forward(packet_rx) & 1) << 2));
        }
	std::shared_ptr<Chunk> rx_chunk = std::make_shared<Chunk>(_payload_len + md_size);
        if(accepted && emergency_rx) {
          memcpy(rx_chunk->data(), md, md_size);
          memcpy(rx_chunk->data() + md_size, sos.data(), sos.size());
          rx_chunk->setSize(md_size + sos.size());
          if(mac_mode) {
            // no SuM header on air: the destination is in the ADB
            MacHdr hdr = {};
            hdr.src_addr = rx_station_id & 0xF;
            hdr.dest_addr = BROADCAST_ADDRESS;
            rx_chunk->addHeader(&hdr, sizeof(hdr));
          }
          pushRxChunk(rx_chunk);
        } else if(accepted && rx_chunk->getSize() <= _payload_len) {
          memcpy(rx_chunk->data(), md, md_size);
          if(mac_mode){
            // Ensure payload has at least 1 byte of application data (excluding MAC header).
            // If needed, this can be relaxed to >= without issues, except for minor processing inefficiency.
            if(_payload_len>sizeof(MacHdr)){
                char hdr[Chunk::MAX_HEADER_SIZE] = {0};
                memcpy(hdr, reinterpret_cast<char *>(
                  janus_packet_get_cargo(packet_rx)), sizeof(MacHdr));
                memcpy(rx_chunk->data() + md_size, reinterpret_cast<char *>(
                  janus_packet_get_cargo(packet_rx)) + sizeof(MacHdr), 
                _payload_len - sizeof(MacHdr));
                rx_chunk->setSize(md_size + _payload_len - sizeof(MacHdr));
                rx_chunk->addHeader(hdr,sizeof(MacHdr));
                pushRxChunk(rx_chunk);
            }
          }else {
            memcpy(rx_chunk->data() + md_size, reinterpret_cast<char *>(
              janus_packet_get_cargo(packet_rx)),_payload_len);
            rx_chunk->setSize(md_size + _payload_len);
            pushRxChunk(rx_chunk);
          }
        }
      } else if (janus_packet_get_cargo_error(packet_rx) != 0) {
        // ERROR janus_packet_get_cargo_error
      } else if (!janus_packet_get_validity(packet_rx)) {
        //Janus RX:: ERROR janus_packet_get_validity
      }
      janus_packet_reset(packet_rx);
      queried_detection_time = 0;
      janus_carrier_sensing_reset(carrier_sensing);
    }
    else
    {     
      demod_data_available -= JanusWrapper::JANUS_RED_SIZE;  
      if(demod_data_available < 0) {        
        demod_data_available = 0;
      }
      double rx_time = (samples_in - demod_data_available
        / params->stream_channel_count) / (double)params->stream_fs;
      lck.unlock();
      bool detected = janus_simple_rx_has_detected(simple_rx);
      if(detected && !queried_detection_time)
      {
        queried_detection_time = 1;
      }
      // libjanus GA detector: > 0 busy window, < 0 window not full yet
      // (startup, after a packet), 0 clear; it is not run while a packet
      // is being decoded, which counts as busy
      int cs = janus_carrier_sensing_execute(carrier_sensing, &time);
      double cs_time = janus_carrier_sensing_time(carrier_sensing);
      if(channel_listener && (detected || cs != 0 || cs_time != last_cs_time)) {
        ChannelEvent ev = {rx_time, detected || cs != 0, detected,
          janus_carrier_sensing_window_power(carrier_sensing),
          janus_carrier_sensing_background_power(carrier_sensing), 0, 0};
        channel_listener(ev);
      }
      last_cs_time = cs_time;
    }
  }
  janus_packet_free(packet_rx);
}
