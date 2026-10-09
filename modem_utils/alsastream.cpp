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

#include "alsastream.h"

#include <iostream>
#include <thread>
#include <algorithm>


const int wait_before_exit = 500; //ms
const std::string AlsaStream::pcm_name = "default";

AlsaStream::AlsaStream()
	: AlsaStream::AlsaStream(pcm_name.c_str(), 96000)
{
}

AlsaStream::AlsaStream(const char * alsa_pcm, unsigned int rate)
	: channels(1)
	, device_name(alsa_pcm)
	, dir(0)
	, rate(rate)
	, rrate(rate)
	, total_frames_sent(0)
	, buffer_time(40000)
	, period_time(1000)
	, resample(1)
	, handle_tx(nullptr)
	, handle_rx(nullptr)
	, format(SND_PCM_FORMAT_S16_LE)
	, buffer_size(0)
	, period_size(0)
	, hw_params(nullptr)
	, sw_params(nullptr)
{
}

AlsaStream::~AlsaStream()
{
	if(handle_rx != nullptr)
		snd_pcm_unlink(handle_rx);
	if(hw_params != nullptr)
		snd_pcm_hw_params_free(hw_params);
	if(sw_params != nullptr)
		snd_pcm_sw_params_free(sw_params);
	if(handle_tx != nullptr) {
		snd_pcm_drop(handle_tx);
		snd_pcm_close(handle_tx);
	}
	if(handle_rx != nullptr) {
		snd_pcm_drop(handle_rx);
		snd_pcm_close(handle_rx);
	}
}

int
AlsaStream::init()
{
	int err = 0;	
	if((err = snd_pcm_open(&handle_tx, device_name.c_str(), SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
		std::cout << "Cannot open device for PLAYBACK with error : " << err << std::endl;
		return err;
	}

	if((err = snd_pcm_open(&handle_rx, device_name.c_str(), SND_PCM_STREAM_CAPTURE, 0)) < 0) {
		std::cout << "Cannot open device for CAPTURE : " << err << std::endl;
	 	return err;
	}

	if((err = snd_pcm_hw_params_malloc(&hw_params)) < 0) {
		std::cout << "snd_pcm_hw_params_malloc error : " << err << std::endl;
		return err;
	}
	memset(hw_params, 0, snd_pcm_hw_params_sizeof());

	if((err = set_hw_params(handle_tx)) < 0) {
		std::cout << "set_hw_params handle_tx error: " << err << std::endl;
		return err;
	}

	if((err = set_hw_params(handle_rx)) < 0) {
		std::cout << "set_hw_params handle_rx error: " << err << std::endl;
		return err;
	}

	if((err = snd_pcm_sw_params_malloc(&sw_params)) < 0) {
		std::cout << "set_sw_params sw_params error: " << err << std::endl;
		return err;
	}
	memset(sw_params, 0, snd_pcm_sw_params_sizeof());

	if((err = set_sw_params(handle_tx)) < 0) {
		std::cout << "set_sw_params handle_tx error: " << err << std::endl;
		return err;
	}

	if((err = set_sw_params(handle_rx)) < 0) {
		std::cout << "set_sw_params handle_rx error: " << err << std::endl;
		return err;
	}

	if ((err = snd_pcm_prepare(handle_tx)) < 0) {
		std::cout << "snd_pcm_prepare handle_tx error: " << err << std::endl;
		return err;
	}

	if ((err = snd_pcm_prepare(handle_rx)) < 0) {
		std::cout << "snd_pcm_prepare handle_rx error: " << err << std::endl;
		return err;
	}

	if ((err = snd_pcm_start(handle_rx)) < 0) {
		std::cout << "snd_pcm_start handle_rx error: " << err << std::endl;
		return err;
	}

	return 0;
}


int
AlsaStream::set_hw_params(snd_pcm_t* handle)
{
	int err = 0;

	if((err = snd_pcm_hw_params_any(handle, hw_params)) < 0) {
		return err;
	}

	if((err = snd_pcm_hw_params_set_rate_resample(handle, hw_params, resample)) < 0) {
		return err;
	}

	if((err = snd_pcm_hw_params_set_access(handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
		return err;
	}

	if((err = snd_pcm_hw_params_set_format(handle, hw_params, format)) < 0) {
		return err;
	}

	if((err = snd_pcm_hw_params_set_channels(handle, hw_params, channels)) < 0) {
		return err;
	}

	if((err = snd_pcm_hw_params_set_rate_near(handle, hw_params, &rrate, 0)) < 0) {
		return err;
	}

	unsigned int temp_buffer_time = buffer_time;
	if((err = snd_pcm_hw_params_set_buffer_time_near(handle, hw_params, &temp_buffer_time, &dir)) < 0) {
		return err;
	}
	unsigned int temp_period_time = period_time;
	if((err = snd_pcm_hw_params_set_period_time_near(handle, hw_params, &temp_period_time, &dir)) < 0) {
		return err;
	}

	if((err = snd_pcm_hw_params_get_period_size(hw_params, &period_size, &dir)) < 0) {
		return err;
	}

	if((err = snd_pcm_hw_params_get_buffer_size(hw_params, &buffer_size)) < 0) {
		return err;
	}

	if((err = snd_pcm_hw_params(handle, hw_params)) < 0) {
		return err;
	}

	return 0;
}


int
AlsaStream::set_sw_params(snd_pcm_t *handle)
{
	int err = 0;
	if((err = snd_pcm_sw_params_current(handle, sw_params)) < 0) {
		return err;
	}

	if((err = snd_pcm_sw_params_set_start_threshold(handle, sw_params, period_size)) < 0) {
		return err;
	}

	if((err = snd_pcm_sw_params_set_avail_min(handle, sw_params, period_size/4)) < 0) {
		return err;
	}

	if((err = snd_pcm_sw_params(handle, sw_params)) < 0) {
		return err;
	}

	return 0;
}


int
AlsaStream::transmit(const std::vector<int16_t> &data)
{
	size_t offset = 0;
	int err = 0;
	unsigned int rrate = 0;
	snd_pcm_hw_params_get_rate(hw_params, &rrate, 0);
	while(offset < data.size()) {
		int rv = 0;
		if(period_size > (data.size() - offset)) {
			int16_t tmp[period_size] = {0};
			for(int i = 0; i < (int)(data.size() - offset); i++) {
				tmp[i] = data[offset+i];
			}			
			snd_pcm_format_set_silence(format, &tmp[data.size() - offset],
											   (period_size - (data.size() - offset)));
			rv = snd_pcm_writei(handle_tx, tmp, period_size);
		} else {
			rv = snd_pcm_writei(handle_tx, &data[offset], period_size);
		}
		if(rv < 0) {
			// Recover transmission to audio interface
			if(rv == -EPIPE) {
				err = snd_pcm_prepare(handle_tx);
			} else if (rv == -ESTRPIPE) {
				while((err = snd_pcm_resume(handle_tx)) == -EAGAIN) {
					std::this_thread::sleep_for(std::chrono::milliseconds(wait_before_exit));
				}
				if(err < 0) { 
					if((err = snd_pcm_prepare(handle_tx)) < 0) {
						return err;
					}
				}
			}
		} else if (rv > 0) {
			offset = offset + rv;	
		}
	}
	total_frames_sent += offset;
	err = 0;
	if((err+=snd_pcm_drain(handle_tx)) == 0) {
        	err+=snd_pcm_prepare(handle_tx);
	}
	if(err!=0) {
		return -1;
	}
	return offset;
}


std::shared_ptr<Samples>
AlsaStream::receive(int num_rec_symbols)
{
	int rv = 0;
	std::shared_ptr<Samples> samples = std::make_shared<Samples>(num_rec_symbols);
	if(samples->samples.size() == 0) {
		return nullptr;
	}
	if((rv = snd_pcm_readi(handle_rx, samples->samples.data(), samples->samples.size())) <= 0) {
		if(rv == -EPIPE) {
			std::cerr << "ALSA RX overrun" << std::endl;
		}
		snd_pcm_prepare(handle_rx);
		rv = snd_pcm_readi(handle_rx, samples->samples.data(), samples->samples.size());
		if(rv <= 0) {
			// sample_size is unsigned: a negative error code would make the
			// demodulator copy megabytes of garbage into the JANUS fifo
			return nullptr;
		}
	}
	samples->sample_size = rv;
	return samples;
}


bool 
AlsaStream::clear_old_rx_data()
{
	if(snd_pcm_drop(handle_rx) >= 0) {
		if(snd_pcm_prepare(handle_rx) >= 0) {
			return true;
		}
	}
	return false;	
}
