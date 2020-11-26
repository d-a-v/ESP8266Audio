/*
  AudioOutputI2S
  Base class for I2S interface port
  
  Copyright (C) 2017  Earle F. Philhower, III

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <i2s.h>
#include "AudioOutputI2S.h"

AudioOutputI2S::AudioOutputI2S(int port, int output_mode, int dma_buf_count, int use_apll)
{
  this->portNo = port;
  this->i2sOn = false;
  this->dma_buf_count = dma_buf_count;
  if (output_mode != EXTERNAL_I2S && output_mode != INTERNAL_DAC && output_mode != INTERNAL_PDM) {
    output_mode = EXTERNAL_I2S;
  }
  this->output_mode = output_mode;
  (void) dma_buf_count;
  (void) use_apll;
  if (!i2sOn) {
    orig_bck = READ_PERI_REG(PERIPHS_IO_MUX_MTDO_U);
    orig_ws = READ_PERI_REG(PERIPHS_IO_MUX_GPIO2_U);
    i2s_begin();
  }
  i2sOn = true;
  mono = false;
  bps = 16;
  channels = 2;
  SetGain(1.0);
  SetRate(44100); // Default
}

AudioOutputI2S::~AudioOutputI2S()
{
  if (i2sOn) i2s_end();
  i2sOn = false;
}

bool AudioOutputI2S::SetPinout(int bclk, int wclk, int dout)
{
  (void) bclk;
  (void) wclk;
  (void) dout;
  return false;
}

bool AudioOutputI2S::SetRate(int hz)
{
  // TODO - have a list of allowable rates from constructor, check them
  this->hertz = hz;
  i2s_set_rate(AdjustI2SRate(hz));
  return true;
}

bool AudioOutputI2S::SetBitsPerSample(int bits)
{
  if ( (bits != 16) && (bits != 8) ) return false;
  this->bps = bits;
  return true;
}

bool AudioOutputI2S::SetChannels(int channels)
{
  if ( (channels < 1) || (channels > 2) ) return false;
  this->channels = channels;
  return true;
}

bool AudioOutputI2S::SetOutputModeMono(bool mono)
{
  this->mono = mono;
  return true;
}

bool AudioOutputI2S::begin()
{
  return true;
}

bool AudioOutputI2S::ConsumeSample(int16_t sample[2])
{
  int16_t ms[2];

  ms[0] = sample[0];
  ms[1] = sample[1];
  MakeSampleStereo16( ms );

  if (this->mono) {
    // Average the two samples and overwrite
    int32_t ttl = ms[LEFTCHANNEL] + ms[RIGHTCHANNEL];
    ms[LEFTCHANNEL] = ms[RIGHTCHANNEL] = (ttl>>1) & 0xffff;
  }
  uint32_t s32 = ((Amplify(ms[RIGHTCHANNEL]))<<16) | (Amplify(ms[LEFTCHANNEL]) & 0xffff);
  return i2s_write_sample_nb(s32); // If we can't store it, return false.  OTW true
}

void AudioOutputI2S::flush() {
}

bool AudioOutputI2S::stop()
{
  return true;
}


