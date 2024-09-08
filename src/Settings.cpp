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

#include "SoapyAirspyHF.hpp"
#include <SoapySDR/Logger.h>
#include <airspyhf.h>
#include <algorithm>

// Driver constructor
SoapyAirspyHF::SoapyAirspyHF(const SoapySDR::Kwargs &args)
    : serial_(0), device_(nullptr), sampleRate_(0), centerFrequency_(0),
      enableDSP_(true), agcEnabled_(true), lnaGain_(0), hfAttenuation_(0),
      frequencyCorrection_(0), iqBalance_(0) {

  // To enable debug logging set the environment variable
  // SOAPY_SDR_LOG_LEVEL to 7. For example:
  // export SOAPY_SDR_LOG_LEVEL=7

  int ret = 0;

  if (args.count("serial")) {
    // For storing serial as hex
    std::stringstream serialstr;

    try {
      // Parse hex to serial number
      serial_ = std::stoull(args.at("serial"), nullptr, 16);
      SoapySDR::logf(SOAPY_SDR_INFO, "Serial number: %016llX", serial_);
    } catch (const std::invalid_argument &) {
      throw std::runtime_error("serial is not a hex number");
    } catch (const std::out_of_range &) {
      throw std::runtime_error("serial value of out range");
    }

    // Serial to hex
    serialstr << std::hex << serial_;
    // Open device
    ret = airspyhf_open_sn(&device_, serial_);
    if (ret != AIRSPYHF_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_open_sn() failed: (%d)", ret);
      throw std::runtime_error("Unable to open AirspyHF device with S/N " +
                               serialstr.str());
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "Found AirspyHF device: serial =  %s",
                   serialstr.str().c_str());
  } else {
    // No serial, open first device
    ret = airspyhf_open(&device_);
    if (ret != AIRSPYHF_SUCCESS) {
      throw std::runtime_error("Unable to open AirspyHF device");
    }
  }

  // TODO: move to some init function
  std::uint32_t num_rates = 0;
  ret = airspyhf_get_samplerates(device_, &num_rates, 0);
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_get_samplerates() failed: (%d)",
                   ret);
  }
  std::vector<uint32_t> rates(num_rates, 0);
  ret = airspyhf_get_samplerates(device_, rates.data(), num_rates);
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_get_samplerates() failed: (%d)",
                   ret);
  }

  // Sort smallest first
  std::sort(rates.begin(), rates.end());

  // Set first sample rate as default
  ret = airspyhf_set_samplerate(device_, rates.front());
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_set_samplerate() failed: (%d)",
                   ret);
  }
  // Store
  sampleRate_ = rates.front();

  // Enables/Disables the IQ Correction, IF shift and Fine Tuning.
  ret = airspyhf_set_lib_dsp(device_, 1);
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_set_lib_dsp() failed: (%d)", ret);
  }
}

SoapyAirspyHF::~SoapyAirspyHF(void) {
  const int ret = airspyhf_close(device_);
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_close() failed: %d", ret);
  }
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyAirspyHF::getDriverKey(void) const { return "AirspyHF"; }

std::string SoapyAirspyHF::getHardwareKey(void) const { return "AirspyHF"; }

SoapySDR::Kwargs SoapyAirspyHF::getHardwareInfo(void) const {
  // key/value pairs for any useful information
  // this also gets printed in --probe
  SoapySDR::Kwargs args;

  std::stringstream serialstr;
  serialstr.str("");
  serialstr << std::hex << serial_;
  args["serial"] = serialstr.str();

  return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyAirspyHF::getNumChannels(const int dir) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getNumChannels(%d)", dir);

  return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string>
SoapyAirspyHF::listAntennas(const int direction, const size_t channel) const {

  std::vector<std::string> antennas;

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "listAntennas(%d, %d)", direction, channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "listAntennas(%d, %d) not supported.",
                   direction, channel);
    return antennas;
  }

  antennas.push_back("RX");

  return antennas;
}

void SoapyAirspyHF::setAntenna(const int direction, const size_t channel,
                               const std::string &name) {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "setAntenna(%d, %d, %s)", direction, channel,
                 name.c_str());

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "setAntenna(%d, %d, %s) not supported.",
                   direction, channel, name.c_str());
    return;
  }
}

std::string SoapyAirspyHF::getAntenna(const int direction,
                                      const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getAntenna(%d, %d)", direction, channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getAntenna(%d, %d) not supported.",
                   direction, channel);
    return {};
  }

  return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyAirspyHF::hasDCOffsetMode(const int direction,
                                    const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "hasDCOffsetMode(%d, %d)", direction,
                 channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "hasDCOffsetMode(%d, %d) not supported.",
                   direction, channel);
    return false;
  }

  return false;
}

bool SoapyAirspyHF::hasIQBalance(const int direction,
                                 const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "hasIQBalance(%d, %d)", direction, channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "hasIQBalance(%d, %d) not supported.",
                   direction, channel);
    return false;
  }

  return true;
}

void SoapyAirspyHF::setIQBalance(const int direction, const size_t channel,
                                 const std::complex<double> &balance) {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "setIQBalance(%d, %d, %f, %f)", direction,
                 channel, balance.real(), balance.imag());

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "setIQBalance(%d, %d) not supported.",
                   direction, channel);
    return;
  }

  if (iqBalance_ != balance) {
    // TODO
    const int ret = airspyhf_set_optimal_iq_correction_point(device_, 0);
    if (ret != AIRSPYHF_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR,
                     "airspyhf_set_optimal_iq_correction_point() failed: %d",
                     ret);
    } else {
      iqBalance_ = balance;
    }
  }
}

std::complex<double> SoapyAirspyHF::getIQBalance(const int direction,
                                                 const size_t channel) const {

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getIQBalance(%d, %d) not supported.",
                   direction, channel);
    return std::complex<double>(0, 0);
  }

  return iqBalance_;
}

bool SoapyAirspyHF::hasFrequencyCorrection(const int direction,
                                           const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "hasFrequencyCorrection(%d, %d)", direction,
                 channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "hasFrequencyCorrection(%d, %d) not supported.", direction,
                   channel);
    return false;
  }

  return true;
}

void SoapyAirspyHF::setFrequencyCorrection(const int direction,
                                           const size_t channel,
                                           const double value) {

  SoapySDR::logf(SOAPY_SDR_DEBUG, "setFrequencyCorrection(%d, %d, %f).",
                 direction, channel, value);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "setFrequencyCorrection(%d, %d) not supported.", direction,
                   channel);
    return;
  }

  // Convert from PPM to PPB
  const int32_t correction_ppb = static_cast<int>(std::round(value * 1000));

  if (frequencyCorrection_ != correction_ppb) {
    int ret = airspyhf_set_calibration(device_, correction_ppb);
    if (ret != AIRSPYHF_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_set_calibration() failed: %d",
                     ret);
    } else {
      frequencyCorrection_ = correction_ppb;
    }
  }
}

double SoapyAirspyHF::getFrequencyCorrection(const int direction,
                                             const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getFrequencyCorrection(%d, %d)", direction,
                 channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "getFrequencyCorrection(%d, %d) not supported.", direction,
                   channel);
    return 0;
  }

  // Convert from PPB to PPM
  return frequencyCorrection_ / 1000.0;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyAirspyHF::listGains(const int direction,
                                                  const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "listGains(%d, %d)", direction, channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "listGains(%d, %d) not supported.",
                   direction, channel);
    return {};
  }

  return {"LNA", "HF_ATT"};
}

bool SoapyAirspyHF::hasGainMode(const int direction,
                                const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "hasGainMode(%d, %d)", direction, channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "hasGainMode(%d, %d) not supported.",
                   direction, channel);
    return false;
  }

  // True means we have an automatic gain mode
  return true;
}

void SoapyAirspyHF::setGainMode(const int direction, const size_t channel,
                                const bool automatic) {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "setGainMode(%d, %d, %d)", direction, channel,
                 automatic);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "setGainMode(%d, %d, %d) not supported.",
                   direction, channel, automatic);
    return;
  }

  if (agcEnabled_ != automatic) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setGainMode(%d, %d, %d)", direction,
                   channel, automatic);
    int ret = airspyhf_set_hf_agc(device_, automatic);
    if (ret != AIRSPYHF_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_set_hf_att() failed: %d", ret);
    } else {
      agcEnabled_ = automatic;
    }
  }
}

bool SoapyAirspyHF::getGainMode(const int direction,
                                const size_t channel) const {

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getGainMode(%d, %d) not supported.",
                   direction, channel);
    return false;
  }

  return agcEnabled_;
}

SoapySDR::Range SoapyAirspyHF::getGainRange(const int direction,
                                            const size_t channel,
                                            const std::string &name) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getGainRange(%d, %d, %s)", direction,
                 channel, name.c_str());

  if (name == "LNA") {
    return SoapySDR::Range(0, 6, 6);
  } else if (name == "HF_ATT") {
    // I think it makes more sense with negative values when it's an attenuator
    // but I'm not sure it will work will all software.
    return SoapySDR::Range(-48, 0, 6);
  } else {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getGainRange(%d, %d, %s) not supported.",
                   direction, channel, name.c_str());
    return SoapySDR::Range(0, 0);
  }
}

double SoapyAirspyHF::getGain(const int direction, const size_t channel,
                              const std::string &name) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getGain(%d, %d, %s)", direction, channel,
                 name.c_str());

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getGain(%d, %d, %s) not supported.",
                   direction, channel, name.c_str());
    return 0;
  }

  if (name == "LNA") {
    return lnaGain_;
  } else if (name == "HF_ATT") {
    return hfAttenuation_;
  } else {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getGain(%d, %d, %s) not supported.",
                   direction, channel, name.c_str());
    return 0;
  }
}

void SoapyAirspyHF::setGain(const int direction, const size_t channel,
                            const double value) {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "setGain(%d, %d, %f)", direction, channel,
                 value);

  // I think the default implementation will try to distribute gain.
  SoapySDR::Device::setGain(direction, channel, value);
}

void SoapyAirspyHF::setGain(const int direction, const size_t channel,
                            const std::string &name, const double value) {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "setGain(%d, %d, %s, %f)", direction, channel,
                 name.c_str(), value);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "setGain(%d, %d, %s, %f) not supported.",
                   direction, channel, name.c_str(), value);
    return;
  }

  if (name == "LNA") {
    const int ret = airspyhf_set_hf_lna(device_, value > 3 ? 1 : 0);
    if (ret != AIRSPYHF_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_set_hf_lna() failed: %d", ret);
    } else {
      lnaGain_ = value;
    }

  } else if (name == "HF_ATT") {
    const uint8_t att = static_cast<uint8_t>(std::round(value / -6));
    const int ret = airspyhf_set_hf_att(device_, att);
    if (ret != AIRSPYHF_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_set_hf_att() failed: %d", ret);
    } else {
      hfAttenuation_ = value;
    }
  } else {
    SoapySDR::logf(SOAPY_SDR_ERROR, "setGain(%d, %d, %s, %f) not supported.",
                   direction, channel, name.c_str(), value);
  }
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyAirspyHF::setFrequency(const int direction, const size_t channel,
                                 const std::string &name,
                                 const double frequency,
                                 const SoapySDR::Kwargs &args) {

  (void)args; // Currently unused. TODO.

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "setFrequency(%d, %d, %s, %f)", direction,
                 channel, name.c_str(), frequency);

  int ret = 0;

  if (direction != SOAPY_SDR_RX or name != "RF" or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "setFrequency(%d, %d, %s, %f) not supported.", direction,
                   channel, name.c_str(), frequency);
    return;
  }

  centerFrequency_ = (uint32_t)frequency;

  SoapySDR::logf(SOAPY_SDR_DEBUG, "setFrequency(%d, %d, %s, %f)", direction,
                 channel, name.c_str(), frequency);

  ret = airspyhf_set_freq(device_, centerFrequency_);
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_set_freq() failed: %d", ret);
  }
}

double SoapyAirspyHF::getFrequency(const int direction, const size_t channel,
                                   const std::string &name) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getFrequency(%d, %d, %s)", direction,
                 channel, name.c_str());

  if (name != "RF") {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getFrequency(%d, %d, %s) not supported.",
                   direction, channel, name.c_str());
    return 0.0;
  }

  return centerFrequency_;
}

std::vector<std::string>
SoapyAirspyHF::listFrequencies(const int direction,
                               const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "listFrequencies(%d, %d)", direction,
                 channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "listFrequencies(%d, %d) not supported.",
                   direction, channel);
    return {};
  }

  return {"RF"};
}

SoapySDR::RangeList
SoapyAirspyHF::getFrequencyRange(const int direction, const size_t channel,
                                 const std::string &name) const {
  SoapySDR::RangeList results;

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getFrequencyRange(%d, %d, %s)", direction,
                 channel, name.c_str());

  if (direction != SOAPY_SDR_RX or name != "RF" or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "getFrequencyRange(%d, %d, %s) not supported.", direction,
                   channel, name.c_str());
    // Empty results
    return results;
  }

  results.push_back(SoapySDR::Range(9'000, 31'000'000)); // 9kHz to 31MHz
  results.push_back(
      SoapySDR::Range(60'000'000, 260'000'000)); // 60MHz to 260MHz

  return results;
}

SoapySDR::ArgInfoList
SoapyAirspyHF::getFrequencyArgsInfo(const int direction,
                                    const size_t channel) const {
  SoapySDR::ArgInfoList freqArgs;

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getFrequencyArgsInfo(%d, %d)", direction,
                 channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "getFrequencyArgsInfo(%d, %d) not supported.", direction,
                   channel);
    return freqArgs;
  }
  // TODO: frequency arguments

  return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyAirspyHF::setSampleRate(const int direction, const size_t channel,
                                  const double rate) {
  int ret = 0;

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "setSampleRate(%d, %d, %f)", direction,
                 channel, rate);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_WARNING,
                   "setSampleRate(%d, %d, %f) not supported.", direction,
                   channel, rate);
    return;
  }

  sampleRate_ = static_cast<uint32_t>(rate);

  ret = airspyhf_set_samplerate(device_, sampleRate_);
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_set_samplerate() failed: %d",
                   ret);
    return;
  }
}

double SoapyAirspyHF::getSampleRate(const int direction,
                                    const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getSampleRate(%d, %d)", direction, channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_WARNING, "getSampleRate(%d, %d) not supported.",
                   direction, channel);
    return 0;
  }

  return sampleRate_;
}

std::vector<double> SoapyAirspyHF::listSampleRates(const int direction,
                                                   const size_t channel) const {

  std::vector<double> results;

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "listSampleRates(%d, %d)", direction,
                 channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "listSampleRates(%d, %d) not supported.",
                   direction, channel);
    return results;
  }

  // Get number of sample rates
  uint32_t numRates = 0;
  int ret = airspyhf_get_samplerates(device_, &numRates, 0);
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_get_samplerates() failed: %d",
                   ret);
    return results;
  }

  // Get sample rates
  std::vector<uint32_t> samplerates(numRates, 0);

  ret = airspyhf_get_samplerates(device_, samplerates.data(), numRates);
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_get_samplerates() failed: %d",
                   ret);
    return results;
  }

  // Sort to make short they are in order
  std::sort(samplerates.begin(), samplerates.end());

  // Use std::copy instead of loop
  std::copy(samplerates.begin(), samplerates.end(),
            std::back_inserter(results));

  return results;
}

void SoapyAirspyHF::setBandwidth(const int direction, const size_t channel,
                                 const double bw) {
  // Log debug
  SoapySDR::logf(SOAPY_SDR_NOTICE, "setBandwidth(%d, %d, %f) not supported",
                 direction, channel, bw);
}

double SoapyAirspyHF::getBandwidth(const int direction,
                                   const size_t channel) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getBandwidth(%d, %d)", direction, channel);

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "getBandwidth(%d, %d) not supported.",
                   direction, channel);
    return 0;
  }

  // TODO: this is just an estimate.
  return 0.9 * sampleRate_;
}

std::vector<double> SoapyAirspyHF::listBandwidths(const int direction,
                                                  const size_t channel) const {
  std::vector<double> results;

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "listBandwidths(%d, %d)", direction, channel);

  const auto sample_rates = listSampleRates(direction, channel);

  std::transform(sample_rates.begin(), sample_rates.end(),
                 std::back_inserter(results),
                 [](const double rate) { return 0.9 * rate; });

  return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyAirspyHF::getSettingInfo(void) const {

  SoapySDR::ArgInfoList setArgs;

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "getSettingInfo()");

  // Enable DSP in library. TODO. make this optional
  SoapySDR::ArgInfo enableDSPArg;
  enableDSPArg.key = "dsp";
  enableDSPArg.value = "true";
  enableDSPArg.name = "DSP";
  enableDSPArg.description = "Enable DSP";
  enableDSPArg.type = SoapySDR::ArgInfo::BOOL;

  return setArgs;
}

void SoapyAirspyHF::writeSetting(const std::string &key,
                                 const std::string &value) {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "writeSetting(%s, %s)", key.c_str(),
                 value.c_str());

  if (key == "dsp") {
    bool enable = (value == "true");

    // Enables/Disables the IQ Correction, IF shift and Fine Tuning.
    const int ret = airspyhf_set_lib_dsp(device_, enable);
    if (ret != AIRSPYHF_SUCCESS) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "airspyhf_set_lib_dsp() failed: (%d)",
                     ret);
    } else {
      SoapySDR::logf(SOAPY_SDR_DEBUG, "airspyhf_set_lib_dsp(%d)", enable);
    }
  } else {
    SoapySDR::logf(SOAPY_SDR_ERROR, "writeSetting(%s, %s) not supported.",
                   key.c_str(), value.c_str());
  }

  SoapySDR::logf(SOAPY_SDR_DEBUG, "writeSetting(%s, %s)", key.c_str(),
                 value.c_str());
}

std::string SoapyAirspyHF::readSetting(const std::string &key) const {

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "readSetting(%s)", key.c_str());

  if (key == "dsp") {
    return enableDSP_ ? "true" : "false";
  } else {
    SoapySDR::logf(SOAPY_SDR_ERROR, "readSetting(%s) not supported.",
                   key.c_str());
    return "";
  }
}
