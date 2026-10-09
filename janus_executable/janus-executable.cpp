/*
 * [2024] SubSeaPulse SRL 
 * All Rights Reserved.
 * 
 * Authors Filippo Campagnaro  
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

#include <iostream>
#include "tcp-stream.h"
#include "alsastream.h"
#include "janus-wrapper.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <gpio.h>
#include <signal.h>
#include <getopt.h>

/** maximum absolute gain for amp output (hw-dependent) */
const float MAX_ABS_GAIN = 0.38;

/** port of the socket used to exchange data with the users */
const int SOCKPORT = 55555;  
/** 
  * When true it ignores the samples received when the modem is transmitting, 
  * so it becomes half duplex.
 */
bool half_duplex{true}; 
bool mac_enabled{false}; /**< when true it adds mac info*/
std::atomic<bool> exit_tx_rx{false}; /**< shared flag to terminate the threads */
std::shared_ptr<ConcBuffer> rx_buffer = std::make_shared<ConcBuffer>(); /**< buffer containing decoded JANUS data */ 
/** 
  * Shared flag to check if the modem is transmitting. 
  * It has an effect only with half_duplex option enabled. 
 */
std::atomic<bool> transmitting{false}; 

TcpStream server{SOCKPORT}; /**< socket used to exchange data with the users */
std::shared_ptr<AlsaStream> astream = nullptr; /**< Alsa object handler */
JanusWrapper janus; /**< Janus modulator/demodulator */
GPIO gpio; /**< GPIO object handler */

MacHdr my_hdr; /**< hdr with source and destination mac */

/**
 * transmission loop, where data received from the user via socket
 * is modulated with JANUS and transmitted via soundcard
 * @param src_addr the source address of this node, just to discard the 
 * packet if it is received  back due to reflection or due to the loopback 
 * of the audio interface. 
 */
void txLoop()
{
  while(!exit_tx_rx.load()) {
    std::shared_ptr<Chunk> tx_chunk = std::make_shared<Chunk>(100);
    if(server.rx(tx_chunk)) {
      gpio.txrxSwitch(true);
      if(mac_enabled) {
        tx_chunk->addHeader(&my_hdr,sizeof(my_hdr));
      }
      transmitting.store(true);
      janus.modulate(tx_chunk);
      astream->transmit(tx_chunk->getModulatedSamples()->samples);
      gpio.txrxSwitch(false);//to check
      transmitting.store(false);
    } 
  }
}

/**
 * loop that receive the samples from the audio interface and gives them to 
 * the demodulator
 */
void rxSamples()
{
  int num_rx_symbols = 4000;
  while(!exit_tx_rx.load()) {
    std::shared_ptr<Samples> samples = nullptr;
    if((samples = astream->receive(num_rx_symbols)) != nullptr // if pointer is valid
        && (!half_duplex || (half_duplex && !transmitting.load()))) { // and either half duplex is disabled or it is not transmitting
      janus.demodulate(samples);
    }
  }
}

/**
 * trivial helper
 */
void printHelp() {
  std::cout << "usage: janus_executable [options]" << std::endl;
  std::cout << "   or: janus_executable fs center_freq bandwidth gain wut half_duplex mac src dest verbose" << std::endl;
  std::cout << "options:" << std::endl;
  std::cout << "  --fs N               sampling frequency (default: 96000)" << std::endl;
  std::cout << "  --center-freq N      central frequency (default: 11520)" << std::endl;
  std::cout << "  --bandwidth N        bandwidth (default: 4160)" << std::endl;
  std::cout << "  --gain X             gain (default: 0.35, [0;1] range)" << std::endl;
  std::cout << "  --wut 0|1            wake-up tones (default: 0) RECEPTION NOT SUPPORTED" << std::endl;
  std::cout << "  --half-duplex 0|1    discard samples received while transmitting (default: 1)" << std::endl;
  std::cout << "  --mac 0|1            proprietary 1-byte src/dest header (default: 0)" << std::endl;
  std::cout << "  --src N              source address (default: 1, range from 1 to 15), used only if MAC enabled" << std::endl;
  std::cout << "  --dest N             destination address (default: 0 (broadcast), range from 1 to 15), need MAC enabled" << std::endl;
  std::cout << "  --verbose N          verbose (default: 0, range from 0 to 3)" << std::endl;
  std::cout << "  --mobility 0|1       baseline packet mobility flag (default: 0 = static)" << std::endl;
  std::cout << "  --tx-rx 0|1          baseline packet tx/rx flag (default: 1 = tx/rx)" << std::endl;
  std::cout << "  --forward 0|1        baseline packet forwarding capability (default: 0)" << std::endl;
  std::cout << "  --class-id N         class user ID (default: 16), a libjanus plugin must exist" << std::endl;
  std::cout << "  --app-type N         application type (default: 0), a libjanus plugin must exist" << std::endl;
  std::cout << "  --station-id N       ADB Station Identifier [0;255] (default: source address)," << std::endl;
  std::cout << "                       received packets with this Station ID are dropped" << std::endl;
  std::cout << "                       (only if MAC enabled or --station-id given)" << std::endl;
  std::cout << "  --pset-id N          ADB Parameter Set Identifier [0;4095] (default: 0), class 16 only" << std::endl;
  std::cout << "  --dest-id N          ADB Destination Identifier [0;255] (default: 255 = broadcast), class 11 only;" << std::endl;
  std::cout << "                       received packets for other destinations are dropped" << std::endl;
  std::cout << "  --rx-class-filter N  accept only this class ID (default: -1 = any)" << std::endl;
  std::cout << "  --rx-app-filter N    accept only this app type (default: -1 = any)" << std::endl;
  std::cout << "  --rx-metadata 0|1    prefix each received payload with 7 bytes (default: 0):" << std::endl;
  std::cout << "                       station_id u8 | dest_id u8 | pset_id u16 BE | class_id u8 | app_type u8 |" << std::endl;
  std::cout << "                       flags u8 (bit0 mobility, bit1 tx_rx, bit2 forward)" << std::endl;
  std::cout << "  --detection-threshold X  preamble detection threshold (default: 3.0, libjanus: 2.5);" << std::endl;
  std::cout << "                       higher gives fewer false detections, lower more range" << std::endl;
  std::cout << "  --help               print this help" << std::endl;
  std::cout << "Text messages (class 11, app type 2 Generic Text Chat, needs libplugin_011_02.so):" << std::endl;
  std::cout << "  text only (no 0x00 bytes), at most 478 bytes, station ID [0;254], MAC header not allowed" << std::endl;
  std::cout << "----------------------------------------------------------------" << std::endl;  
  std::cout << "example:" << std::endl;
  std::cout << "./janus_executable 96000 11520 4160 0.3 0 1 0 1 0 0" << std::endl;
  std::cout << "./janus_executable --fs 96000 --center-freq 14080 --bandwidth 4160 --station-id 5 --rx-metadata 1" << std::endl;
  std::cout << "./janus_executable --center-freq 14080 --bandwidth 4160 --class-id 11 --app-type 2 --station-id 3 --dest-id 255" << std::endl;
  std::cout << "----------------------------------------------------------------" << std::endl; 
  std::cout << "Here the Janus bands:" << std::endl;
  std::cout << " - A: centeral frequency = 11520 bandwidth = 4160" << std::endl;
  std::cout << " - B: centeral frequency = 6000  bandwidth = 2080" << std::endl;
  std::cout << " - C: centeral frequency = 9700  bandwidth = 2600" << std::endl;
  std::cout << " - D: centeral frequency = 14080 bandwidth = 4160" << std::endl;
  std::cout << " - E: centeral frequency = 28000 bandwidth = 6500" << std::endl;
  std::cout << "NB: for band E you nedd a sampling frequency of 96000" << std::endl;
}

/**
 * configuration from the command line
 */
struct Config {
  int sampling_frequency = 96000;
  int center_frequency = 11520;
  int bandwidth = 4160;
  float tx_gain = 0.35;
  int wake_up_tones = 0;
  unsigned int verbose = 0;
  int mobility = 0;
  int tx_rx = 1;
  int forward = 0;
  int class_id = 16;
  int app_type = 0;
  int station_id = -1; /**< -1 means equal to the source address */
  int pset_id = 0;
  int dest_id = 255; /**< broadcast */
  int rx_class_filter = -1;
  int rx_app_filter = -1;
  int rx_metadata = 0;
  double detection_threshold = 3.0;
  bool self_filter = false; /**< drop own packets, on with MAC or explicit station ID */
};

/**
 * parse the command line, either the legacy 10 positional parameters
 * or the named options
 * @return 0 on success, 1 if help was requested, -1 on error
 */
int parseArgs(int argc, char* argv[], Config& cfg)
{
  int src_addr = my_hdr.src_addr;
  int dest_addr = my_hdr.dest_addr;
  if(argc == 11 && argv[1][0] != '-') {
    cfg.sampling_frequency = atoi(argv[1]);
    cfg.center_frequency = atoi(argv[2]);
    cfg.bandwidth = atoi(argv[3]);
    cfg.tx_gain = atof(argv[4]);
    cfg.wake_up_tones = atoi(argv[5]);
    half_duplex = atoi(argv[6]);
    mac_enabled = atoi(argv[7]);
    src_addr = atoi(argv[8]);
    dest_addr = atoi(argv[9]);
    cfg.verbose = atoi(argv[10]);
  }
  else {
    static const struct option long_opts[] = {
      {"fs", required_argument, nullptr, 'f'},
      {"center-freq", required_argument, nullptr, 'c'},
      {"bandwidth", required_argument, nullptr, 'b'},
      {"gain", required_argument, nullptr, 'g'},
      {"wut", required_argument, nullptr, 'w'},
      {"half-duplex", required_argument, nullptr, 'd'},
      {"mac", required_argument, nullptr, 'm'},
      {"src", required_argument, nullptr, 's'},
      {"dest", required_argument, nullptr, 't'},
      {"verbose", required_argument, nullptr, 'v'},
      {"mobility", required_argument, nullptr, 'M'},
      {"tx-rx", required_argument, nullptr, 'R'},
      {"forward", required_argument, nullptr, 'F'},
      {"class-id", required_argument, nullptr, 'C'},
      {"app-type", required_argument, nullptr, 'A'},
      {"station-id", required_argument, nullptr, 'S'},
      {"pset-id", required_argument, nullptr, 'P'},
      {"dest-id", required_argument, nullptr, 'D'},
      {"rx-class-filter", required_argument, nullptr, 'x'},
      {"rx-app-filter", required_argument, nullptr, 'y'},
      {"rx-metadata", required_argument, nullptr, 'z'},
      {"detection-threshold", required_argument, nullptr, 'T'},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, 0, nullptr, 0}
    };
    int opt;
    while((opt = getopt_long(argc, argv, "h", long_opts, nullptr)) != -1) {
      switch(opt) {
        case 'f': cfg.sampling_frequency = atoi(optarg); break;
        case 'c': cfg.center_frequency = atoi(optarg); break;
        case 'b': cfg.bandwidth = atoi(optarg); break;
        case 'g': cfg.tx_gain = atof(optarg); break;
        case 'w': cfg.wake_up_tones = atoi(optarg); break;
        case 'd': half_duplex = atoi(optarg); break;
        case 'm': mac_enabled = atoi(optarg); break;
        case 's': src_addr = atoi(optarg); break;
        case 't': dest_addr = atoi(optarg); break;
        case 'v': cfg.verbose = atoi(optarg); break;
        case 'M': cfg.mobility = atoi(optarg); break;
        case 'R': cfg.tx_rx = atoi(optarg); break;
        case 'F': cfg.forward = atoi(optarg); break;
        case 'C': cfg.class_id = atoi(optarg); break;
        case 'A': cfg.app_type = atoi(optarg); break;
        case 'S': cfg.station_id = atoi(optarg); break;
        case 'P': cfg.pset_id = atoi(optarg); break;
        case 'D': cfg.dest_id = atoi(optarg); break;
        case 'x': cfg.rx_class_filter = atoi(optarg); break;
        case 'y': cfg.rx_app_filter = atoi(optarg); break;
        case 'z': cfg.rx_metadata = atoi(optarg); break;
        case 'T': cfg.detection_threshold = atof(optarg); break;
        case 'h': return 1;
        default: return -1;
      }
    }
    if(optind != argc) {
      return -1;
    }
  }
  if(cfg.tx_gain > 1) {
    std::cout << "provided tx_gain out of range, now set at 1" << std::endl;
    cfg.tx_gain = 1;
  }
  else if(cfg.tx_gain < 0) {
    std::cout << "provided negative tx_gain, now set at 0" << std::endl;
    cfg.tx_gain = 0;
  }
  if(cfg.wake_up_tones){
    printf("WARNING: RECEPTION OF WAKE-UP-TONES NOT SUPPORTED\n");
  }
  my_hdr.src_addr = src_addr;
  my_hdr.dest_addr = dest_addr;
  cfg.self_filter = mac_enabled || cfg.station_id >= 0;
  if(cfg.station_id < 0) {
    cfg.station_id = my_hdr.src_addr;
  }
  return 0;
}

void
signalHandler(int signum)
{
  if (signum == SIGTERM || signum == SIGINT) {

    std::string dbg_msg = std::string("SIGNAL_RECEIVED::")
      + std::to_string(signum);
    std::cout << dbg_msg << std::endl;

    exit_tx_rx.store(true);
    rx_buffer->forceExit();
    return;

  } else {

    std::string err_msg = std::string("SIGNAL_RECEIVED::")
      + std::to_string(signum);
    std::cout << err_msg << std::endl;

    exit(EXIT_FAILURE);
  }
}

/**
 * main function
 */
int main(int argc, char* argv[])
{
  /**
   * signal handling
   */
  struct sigaction act;
  memset(&act, '\0', sizeof(act));
  act.sa_handler = SIG_IGN;

  if (sigaction(SIGPIPE, &act, NULL) < 0) {
    std::cerr << "SIGPIPE" << std::endl;
  }

  memset(&act, '\0', sizeof(act));
  act.sa_handler = &signalHandler;

  if (sigaction(SIGINT, &act, NULL) < 0) {
    std::cerr << "SIGINT" << std::endl;
  } else if (sigaction(SIGILL, &act, NULL) < 0) {
    std::cerr << "SIGILL" << std::endl;
  } else if (sigaction(SIGFPE, &act, NULL) < 0) {
    std::cerr << "SIGFPE" << std::endl;
  } else if (sigaction(SIGSEGV, &act, NULL) < 0) {
    std::cerr << "SIGSEGV" << std::endl;
  } else if (sigaction(SIGBUS, &act, NULL) < 0) {
    std::cerr << "SIGBUS" << std::endl;
  } else if (sigaction(SIGSYS, &act, NULL) < 0) {
    std::cerr << "SIGSYS" << std::endl;
  } else if (sigaction(SIGABRT, &act, NULL) < 0) {
    std::cerr << "SIGABRT" << std::endl;
  } else if (sigaction(SIGALRM, &act, NULL) < 0) {
    std::cerr << "SIGALRM" << std::endl;
  } else if (sigaction(SIGTERM, &act, NULL) < 0) {
    std::cerr << "SIGTERM" << std::endl;
  } else if (sigaction(SIGQUIT, &act, NULL) < 0) {
    std::cerr << "SIGQUIT" << std::endl;
  }


  /**
   * configure the parameters
   */
  my_hdr.src_addr = 1;
  my_hdr.dest_addr = 0;
  Config cfg;
  int parse_rv = parseArgs(argc, argv, cfg);
  if(parse_rv != 0) {
    printHelp();
    return parse_rv > 0 ? 0 : -1;
  }

  std::cout << "sampling_frequency = " << cfg.sampling_frequency 
    <<" center_frequency = " << cfg.center_frequency 
    <<" bandwidth = " << cfg.bandwidth << " tx_gain = " << cfg.tx_gain 
    << " wake_up_tones = " << cfg.wake_up_tones 
    <<" half_duplex = " << half_duplex
    <<" mac_enabled = " << mac_enabled
    <<" src_addr = " << (int)my_hdr.src_addr 
    << " dest_addr = " << (int)my_hdr.dest_addr
    << std::endl;
  std::cout << "mobility = " << cfg.mobility
    << " tx_rx = " << cfg.tx_rx
    << " forward = " << cfg.forward
    << " class_id = " << cfg.class_id
    << " app_type = " << cfg.app_type
    << " station_id = " << cfg.station_id
    << " pset_id = " << cfg.pset_id
    << " dest_id = " << cfg.dest_id
    << " rx_class_filter = " << cfg.rx_class_filter
    << " rx_app_filter = " << cfg.rx_app_filter
    << " rx_metadata = " << cfg.rx_metadata
    << " detection_threshold = " << cfg.detection_threshold
    << std::endl;

  /**
   * Configure the JANUS packet fields before touching the hardware
   */
  if(!janus.setMobility(cfg.mobility) || !janus.setTxRx(cfg.tx_rx)
    || !janus.setForward(cfg.forward) || !janus.setClassId(cfg.class_id)
    || !janus.setAppType(cfg.app_type) || !janus.setStationId(cfg.station_id)
    || !janus.setAdbPsetId(cfg.pset_id) || !janus.setDestId(cfg.dest_id)
    || !janus.setRxFilter(cfg.rx_class_filter, cfg.rx_app_filter)
    || !janus.setDetectionThreshold(cfg.detection_threshold)) {
    std::cerr << "ERROR: JANUS packet field out of range" << std::endl;
    printHelp();
    return -1;
  }
  if(cfg.class_id == JanusWrapper::TEXT_CLASS_ID) {
    // the destination is in the ADB, a MAC header would end up in the text
    if(mac_enabled) {
      std::cerr << "ERROR: MAC header not allowed with class 11, use --dest-id" << std::endl;
      return -1;
    }
    if(cfg.station_id == JanusWrapper::BROADCAST_ID) {
      std::cerr << "ERROR: station ID 255 not allowed with class 11" << std::endl;
      return -1;
    }
  }
  janus.setSelfFilter(cfg.self_filter);
  janus.setRxMetadata(cfg.rx_metadata);

  /**
   * Open Alsa
   */
  astream = std::make_shared<AlsaStream>("default", cfg.sampling_frequency);
  astream->init();

  /**
   * Configure JANUS
   */
  janus.setSamplingFrequency(cfg.sampling_frequency);
  janus.setCarrierFrequency(cfg.center_frequency);
  janus.setBandwidth(cfg.bandwidth);
  janus.setTxGain(cfg.tx_gain * MAX_ABS_GAIN);  
  janus.setWakeUpTones(cfg.wake_up_tones);
  janus.setMacMode(mac_enabled);  
  /**
   * reception buffer shared between the demodulator (Janus receiver) and the 
   * loop that sends the received data to the user
   */
  janus.setRxBuffer(rx_buffer);
  janus.setVerbose(cfg.verbose);
  if(!janus.janusSetup()) {
    std::cerr << "ERROR: JANUS setup failed, unsupported class ID / app type" << std::endl;
    return -1;
  }

  /**
   * Start the thread running the the transmission loop
   */
  std::thread listen_thread(&txLoop); 

  /**
   * Start the thread running the the loop that reads the samples 
   * and gives them to the demodulator
   */
  std::thread rx_samples_thread(&rxSamples);

  /**
   * rx loop that sends the received data to the user
   */
  MacHdr tx_hdr = {};
  while(!exit_tx_rx.load()) {
    std::shared_ptr<Chunk> rx_chunk = nullptr;
    if(rx_buffer->wPop(rx_chunk)){
      if(mac_enabled) {
        // self interference is already dropped by the wrapper (Station ID)
        rx_chunk->deserializeHeader(&tx_hdr,sizeof(tx_hdr));
        if(tx_hdr.dest_addr == BROADCAST_ADDRESS || tx_hdr.dest_addr == my_hdr.src_addr) {
          server.tx(rx_chunk);
        }
      } else { // if MAC is disabled
        server.tx(rx_chunk);
      }
    }
  }
  server.quit();
  listen_thread.join();
  rx_samples_thread.join();
  exit(EXIT_SUCCESS);
}
