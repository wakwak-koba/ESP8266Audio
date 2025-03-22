/*
  AudioOutputSPDIF
  
  S/PDIF output via I2S
  
  Needs transceiver from CMOS level to either optical or coaxial interface
  See: https://www.epanorama.net/documents/audio/spdif.html

  Original idea and sources: 
    Forum thread discussing implementation
      https://forum.pjrc.com/threads/28639-S-pdif
    Teensy Audio Library 
      https://github.com/PaulStoffregen/Audio/blob/master/output_spdif2.cpp
   
  Adapted for ESP8266Audio

  Copyright (C) 2020 Ivan Kostoski

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

#if defined(ESP32) || defined(ESP8266)
#if defined(ESP32) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #include "driver/i2s_std.h"
#endif
#pragma once

#include "AudioOutput.h"

#if defined(ESP32)
#ifdef CONFIG_IDF_TARGET_ESP32S3
#define SPDIF_OUT_PIN_DEFAULT  1
#else
#define SPDIF_OUT_PIN_DEFAULT  27
#endif
#define DMA_BUF_COUNT_DEFAULT  8
#define DMA_BUF_SIZE_DEFAULT   256
#elif defined(ESP8266)
#define SPDIF_OUT_PIN_DEFAULT  3
#define DMA_BUF_COUNT_DEFAULT  32
#define DMA_BUF_SIZE_DEFAULT   64
#endif

class AudioOutputSPDIF : public AudioOutput
{
  public:
    AudioOutputSPDIF(int dout_pin=SPDIF_OUT_PIN_DEFAULT, int port=0, int dma_buf_count = DMA_BUF_COUNT_DEFAULT);
    virtual ~AudioOutputSPDIF() override;
    bool SetPinout(int doutPin);
    virtual bool SetRate(int hz) override;
    virtual bool SetBitsPerSample(int bits) override;
    virtual bool SetChannels(int channels) override;
    virtual bool begin() override;
    virtual bool ConsumeSample(int16_t sample[2]) override;
    virtual bool stop() override;

    bool SetOutputModeMono(bool mono);  // Force mono output no matter the input

    const uint32_t VUCP_PREAMBLE_B = 0xCCE80000; // 11001100 11101000
    const uint32_t VUCP_PREAMBLE_M = 0xCCE20000; // 11001100 11100010
    const uint32_t VUCP_PREAMBLE_W = 0xCCE40000; // 11001100 11100100

  protected:
    virtual inline int AdjustI2SRate(int hz) { return rate_multiplier * hz; }
    uint8_t portNo;
    bool mono;
    bool i2sOn;
    int dma_buf_count;
    uint8_t frame_num;
    uint8_t rate_multiplier;
    uint8_t doutPin;

  #ifdef ESP32
  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    i2s_chan_handle_t tx_handle = nullptr;
  #endif
#endif
};

#endif // _AUDIOOUTPUTSPDIF_H
