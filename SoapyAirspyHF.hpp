/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Charles J. Cliffe
 * Copyright (c) 2018 Corey Stotts
 * Copyright (c) 2022 Albin Stigö

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#pragma once

#include <SoapySDR/ConverterRegistry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Types.hpp>

#include <algorithm>
#include <atomic>
#include <complex>
#include <cstring>
#include <stdexcept>
#include <string>

#include <libairspyhf/airspyhf.h>

#include "RingBuffer.hpp"

#define MAX_DEVICES 32

class SoapyAirspyHF : public SoapySDR::Device {
private:
  // Device handle
  uint64_t serial_;
  airspyhf_device_t *dev_;

  uint32_t sampleRate_;
  uint32_t centerFrequency_;

  bool enableDSP_;
  bool agcEnabled_;
  double lnaGain_;
  double hfAttenuation_;

  double frequencyCorrection_;
  std::complex<double> iqBalance_;

  SoapySDR::ConverterRegistry::ConverterFunction converterFunction_;

  std::atomic<long long> ticks_;
  RingBuffer<airspyhf_complex_float_t> ringbuffer_;

  friend int rx_callback_(airspyhf_transfer_t *transfer);
  int rx_callback(airspyhf_transfer_t *t);

public:
  SoapyAirspyHF(const SoapySDR::Kwargs &args);
  ~SoapyAirspyHF(void);

  SoapyAirspyHF(const SoapyAirspyHF &) = delete;
  SoapyAirspyHF &operator=(const SoapyAirspyHF &) = delete;

  /*******************************************************************
   * Identification API
   ******************************************************************/

  std::string getDriverKey(void) const override;

  std::string getHardwareKey(void) const override;

  SoapySDR::Kwargs getHardwareInfo(void) const override;

  /*******************************************************************
   * Channels API
   ******************************************************************/

  size_t getNumChannels(const int) const override;

  /*******************************************************************
   * Stream API
   ******************************************************************/

  std::vector<std::string>
  getStreamFormats(const int direction, const size_t channel) const override;

  std::string getNativeStreamFormat(const int direction, const size_t channel,
                                    double &fullScale) const override;

  SoapySDR::ArgInfoList getStreamArgsInfo(const int direction,
                                          const size_t channel) const override;

  SoapySDR::Stream *
  setupStream(const int direction, const std::string &format,
              const std::vector<size_t> &channels = std::vector<size_t>(),
              const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

  void closeStream(SoapySDR::Stream *stream) override;

  size_t getStreamMTU(SoapySDR::Stream *stream) const override;

  int activateStream(SoapySDR::Stream *stream, const int flags = 0,
                     const long long timeNs = 0,
                     const size_t numElems = 0) override;

  int deactivateStream(SoapySDR::Stream *stream, const int flags = 0,
                       const long long timeNs = 0) override;

  int readStream(SoapySDR::Stream *stream, void *const *buffs,
                 const size_t numElems, int &flags, long long &timeNs,
                 const long timeoutUs = 100000) override;

  /*******************************************************************
   * Antenna API
   ******************************************************************/

  std::vector<std::string> listAntennas(const int direction,
                                        const size_t channel) const override;

  void setAntenna(const int direction, const size_t channel,
                  const std::string &name) override;

  std::string getAntenna(const int direction,
                         const size_t channel) const override;

  /*******************************************************************
   * Frontend corrections API
   ******************************************************************/

  bool hasDCOffsetMode(const int direction,
                       const size_t channel) const override;

  //  void setDCOffsetMode(const int direction, const size_t channel, const bool
  //  automatic); bool getDCOffsetMode(const int direction, const size_t
  //  channel) const; bool hasDCOffset(const int direction, const size_t
  //  channel) const; void setDCOffset(const int direction, const size_t
  //  channel, const std::complex<double> &offset);
  //   std::complex<double> getDCOffset(const int direction, const size_t
  //   channel) const;

  bool hasIQBalance(const int direction, const size_t channel) const override;

  void setIQBalance(const int direction, const size_t channel,
                    const std::complex<double> &balance) override;

  virtual std::complex<double>
  getIQBalance(const int direction, const size_t channel) const override;

  // virtual bool hasIQBalanceMode(const int direction, const size_t channel)
  // const; virtual void setIQBalanceMode(const int direction, const size_t
  // channel, const bool automatic); virtual bool getIQBalanceMode(const int
  // direction, const size_t channel) const;

  bool hasFrequencyCorrection(const int direction,
                              const size_t channel) const override;

  void setFrequencyCorrection(const int direction, const size_t channel,
                              const double value) override;

  double getFrequencyCorrection(const int direction,
                                const size_t channel) const override;

  /*******************************************************************
   * Gain API
   ******************************************************************/

  std::vector<std::string> listGains(const int direction,
                                     const size_t channel) const override;

  bool hasGainMode(const int direction, const size_t channel) const override;

  void setGainMode(const int direction, const size_t channel,
                   const bool automatic) override;

  bool getGainMode(const int direction, const size_t channel) const override;

  void setGain(const int direction, const size_t channel,
               const double value) override;

  void setGain(const int direction, const size_t channel,
               const std::string &name, const double value) override;

  double getGain(const int direction, const size_t channel,
                 const std::string &name) const override;

  SoapySDR::Range getGainRange(const int direction, const size_t channel,
                               const std::string &name) const override;

  /*******************************************************************
   * Frequency API
   ******************************************************************/

  void setFrequency(const int direction, const size_t channel,
                    const std::string &name, const double frequency,
                    const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

  double getFrequency(const int direction, const size_t channel,
                      const std::string &name) const override;

  std::vector<std::string> listFrequencies(const int direction,
                                           const size_t channel) const override;

  SoapySDR::RangeList getFrequencyRange(const int direction,
                                        const size_t channel,
                                        const std::string &name) const override;

  SoapySDR::ArgInfoList
  getFrequencyArgsInfo(const int direction,
                       const size_t channel) const override;

  /*******************************************************************
   * Sample Rate API
   ******************************************************************/

  void setSampleRate(const int direction, const size_t channel,
                     const double rate) override;

  double getSampleRate(const int direction,
                       const size_t channel) const override;

  std::vector<double> listSampleRates(const int direction,
                                      const size_t channel) const override;

  void setBandwidth(const int direction, const size_t channel,
                    const double bw) override;

  double getBandwidth(const int direction, const size_t channel) const override;

  std::vector<double> listBandwidths(const int direction,
                                     const size_t channel) const override;

  /*******************************************************************
   * Utility
   ******************************************************************/

  /*******************************************************************
   * Settings API
   ******************************************************************/

  SoapySDR::ArgInfoList getSettingInfo(void) const override;

  void writeSetting(const std::string &key, const std::string &value) override;

  std::string readSetting(const std::string &key) const override;
};
