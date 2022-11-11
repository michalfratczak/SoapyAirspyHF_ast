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


SoapyAirspyHF::SoapyAirspyHF(const SoapySDR::Kwargs &args)
    : dev_(nullptr),
      sampleRate_(0),
      frequencyCorrection_(0),
      iqBalance_(0),
      ringbuffer_(8 * 2048) // Keep 8 buffers in the ringbuffer
{

    int ret;

    SoapySDR_setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_DEBUG);

    if (args.count("serial") != 0)
    {
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
        ret = airspyhf_open_sn(&dev_, serial_);
        if (ret != AIRSPYHF_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_open_sn() failed: (%d)", ret);
            throw std::runtime_error("Unable to open AirspyHF device with S/N " + serialstr.str());
        }

        SoapySDR::logf(SOAPY_SDR_INFO, "Found AirspyHF device: serial =  %s",
                       serialstr.str().c_str());
    }
    else
    {
        // No serial, open first device
        ret = airspyhf_open(&dev_);
        if (ret != AIRSPYHF_SUCCESS) {
            throw std::runtime_error("Unable to open AirspyHF device");
        }
    }

    // Set first sample rate as default
    auto const& rates = listSampleRates(SOAPY_SDR_RX, 0);

    setSampleRate(SOAPY_SDR_RX, 0, rates.front());
    setFrequency(SOAPY_SDR_RX, 0, "RF", 7000000);
    // Default to no gain
    setGain(SOAPY_SDR_RX, 0, 0);
    // And no AGC
    setGainMode(SOAPY_SDR_RX, 0, false);

    // Enables/Disables the IQ Correction, IF shift and Fine Tuning.
    ret = airspyhf_set_lib_dsp(dev_, 1);
    if(ret != AIRSPYHF_SUCCESS) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_set_lib_dsp() failed: (%d)", ret);
    }

    // Apply arguments to settings when they match
    for (const auto &info : getSettingInfo()) {
        const auto it = args.find(info.key);
        if (it != args.end()) {
            writeSetting(it->first, it->second);
        }
    }
}

SoapyAirspyHF::~SoapyAirspyHF(void)
{
    int ret;

    ret = airspyhf_close(dev_);
    if(ret != AIRSPYHF_SUCCESS) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_close() failed: %d", ret);
    }
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyAirspyHF::getDriverKey(void) const
{
    return "AirspyHF";
}

std::string SoapyAirspyHF::getHardwareKey(void) const
{
    return "AirspyHF";
}

SoapySDR::Kwargs SoapyAirspyHF::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
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

size_t SoapyAirspyHF::getNumChannels(const int dir) const
{
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyAirspyHF::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;

    antennas.push_back("RX");

    return antennas;
}

void SoapyAirspyHF::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setAntenna(%d, %d, %s) not supported.", direction, channel, name.c_str());
}

std::string SoapyAirspyHF::getAntenna(const int direction, const size_t channel) const
{
    // I think this switch is automatically handled by libairpsyhf
    return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

// bool SoapyAirspyHF::hasDCOffsetMode(const int direction, const size_t channel) const
// {
//     return false;
//}

bool SoapyAirspyHF::hasIQBalance(const int direction, const size_t channel) const {
    return true;
}

void SoapyAirspyHF::setIQBalance(const int direction, const size_t channel,
                                 const std::complex<double> &balance) {

    int ret;

    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setIQBalance(%d, %d) not supported.", direction, channel);
        return;
    }

    if(iqBalance_ != balance) {
        // TODO
        ret = airspyhf_set_optimal_iq_correction_point(dev_, 0);
        if(ret != AIRSPYHF_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_set_optimal_iq_correction_point() failed: %d", ret);
        } else {
            iqBalance_ = balance;
        }
    }
}

std::complex<double> SoapyAirspyHF::getIQBalance(const int direction,
                                                 const size_t channel) const {
    return iqBalance_;
}

bool SoapyAirspyHF::hasFrequencyCorrection(const int direction,
                                           const size_t channel) const {
    return true;
}

void SoapyAirspyHF::setFrequencyCorrection(const int direction,
                                           const size_t channel,
                                           const double value) {

  SoapySDR_logf(SOAPY_SDR_DEBUG, "setFrequencyCorrection(%d, %d, %f).", direction, channel, value);

    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setFrequencyCorrection(%d, %d) not supported.", direction, channel);
        return;
    }

    const int32_t correction_ppb =  value * 1000;
    if(frequencyCorrection_ != correction_ppb) {
        int ret  = airspyhf_set_calibration(dev_, correction_ppb);
        if(ret != AIRSPYHF_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_set_calibration() failed: %d", ret);
        } else {
            frequencyCorrection_ = correction_ppb;
        }
    }
}

double SoapyAirspyHF::getFrequencyCorrection(const int direction,
                                             const size_t channel) const {
    return 0;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyAirspyHF::listGains(const int direction, const size_t channel) const
{
    std::vector<std::string> results;

    results.push_back("LNA");
    results.push_back("HF ATT");

    return results;
}

bool SoapyAirspyHF::hasGainMode(const int direction, const size_t channel) const
{
    // True means we have an automatic gain mode
    return true;
}

void SoapyAirspyHF::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setGainMode(%d, %d, %d) not supported.", direction, channel, automatic);
        return;
    }

    if(agcEnabled_ != automatic) {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "setGainMode(%d, %d, %d)", direction, channel, automatic);
        int ret = airspyhf_set_hf_agc(dev_, automatic);
        if(ret != AIRSPYHF_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_set_hf_att() failed: %d", ret);
        } else {
            agcEnabled_ = automatic;
        }
    }
}

bool SoapyAirspyHF::getGainMode(const int direction, const size_t channel) const
{
    return agcEnabled_;
}

SoapySDR::Range SoapyAirspyHF::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "LNA") {
        return SoapySDR::Range(0, 6, 6);
    }
    else if (name == "HF ATT") {
        return SoapySDR::Range(0, 48, 6);
    }
    else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getGainRange(%d, %d, %s) not supported.", direction, channel, name.c_str());
        return SoapySDR::Range(0, 0);
    }
}

double SoapyAirspyHF::getGain(const int direction, const size_t channel, const std::string &name) const
{
    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getGain(%d, %d, %s) not supported.", direction, channel, name.c_str());
        return 0;
    }

    if (name == "LNA") {
        return lnaGain_;
    }
    else if (name == "HF ATT") {
        return hfAttenuation_;
    }
    else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getGain(%d, %d, %s) not supported.", direction, channel, name.c_str());
        return 0;
    }
}

void SoapyAirspyHF::setGain(const int direction, const size_t channel, const double value) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setGain(%d, %d, %f) not supported", direction, channel, value);
}

void SoapyAirspyHF::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setGain(%d, %d, %s, %f) not supported.", direction, channel, name.c_str(), value);
        return;
    }

    int ret;

    if (name == "LNA") {
        ret = airspyhf_set_hf_lna(dev_, value > 3 ? 1 : 0);
        if(ret != AIRSPYHF_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_set_hf_lna() failed: %d", ret);
        } else {
            lnaGain_ = value;
        }

    }
    else if (name == "HF ATT") {
        const uint8_t att = static_cast<uint8_t>(std::round(value / 6));
        SoapySDR_logf(SOAPY_SDR_DEBUG, "setGain(%d, %d, %s, %f) -> %d",
                      direction, channel, name.c_str(), value, att);
        int ret = airspyhf_set_hf_att(dev_, att);
        if(ret != AIRSPYHF_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_set_hf_att() failed: %d", ret);
        } else {
            hfAttenuation_ = value;
        }
    }
    else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setGain(%d, %d, %s, %f) not supported.", direction, channel, name.c_str(), value);
    }
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyAirspyHF::setFrequency(const int direction,
                                 const size_t channel,
                                 const std::string &name,
                                 const double frequency,
                                 const SoapySDR::Kwargs &args) {
    int ret;

    if(direction != SOAPY_SDR_RX or name != "RF" or channel != 0) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setFrequency(%d, %d, %s, %f) not supported.",
                      direction, channel, name.c_str(), frequency);
        return;
    }

    centerFrequency_ = (uint32_t)frequency;

    SoapySDR_logf(SOAPY_SDR_DEBUG, "setFrequency(%d, %d, %s, %f)",
                  direction, channel, name.c_str(), frequency);

    ret = airspyhf_set_freq(dev_, centerFrequency_);
    if(ret != AIRSPYHF_SUCCESS) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_set_freq() failed: %d", ret);
    }
}

double SoapyAirspyHF::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    if(name != "RF") {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequency(%d, %d, %s) not supported.",
                      direction, channel, name.c_str());
        return 0.0;
    }

    return centerFrequency_;
}

std::vector<std::string> SoapyAirspyHF::listFrequencies(const int direction,
                                                        const size_t channel) const
{
    std::vector<std::string> names;

    names.push_back("RF");

    return names;
}

SoapySDR::RangeList SoapyAirspyHF::getFrequencyRange(const int direction,
                                                     const size_t channel,
                                                     const std::string &name) const
{
    SoapySDR::RangeList results;

    if(direction != SOAPY_SDR_RX or name != "RF" or channel != 0) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequencyRange(%d, %d, %s) not supported.",
                      direction, channel, name.c_str());
        // Empty results
        return results;
    }

    results.push_back(SoapySDR::Range(9000, 31000000)); // 9kHz to 31MHz
    results.push_back(SoapySDR::Range(60000000, 260000000)); // 60MHz to 260MHz

    return results;
}

SoapySDR::ArgInfoList SoapyAirspyHF::getFrequencyArgsInfo(const int direction,
                                                          const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;

    // TODO: frequency arguments

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyAirspyHF::setSampleRate(const int direction, const size_t channel,
                                  const double rate)
{
    int ret;

    sampleRate_ = (uint32_t)rate;

    ret = airspyhf_set_samplerate(dev_, sampleRate_);
    if(ret != AIRSPYHF_SUCCESS) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_set_samplerate() failed: %d", ret);
        return;
    }
}

double SoapyAirspyHF::getSampleRate(const int direction, const size_t channel) const
{
    return sampleRate_;
}

std::vector<double> SoapyAirspyHF::listSampleRates(const int direction,
                                                   const size_t channel) const
{
    int ret;
    std::vector<double> results;

    uint32_t numRates = 0;
    ret = airspyhf_get_samplerates(dev_, &numRates, 0);
    if (ret != AIRSPYHF_SUCCESS) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_get_samplerates() failed: %d", ret);
        return results;
    }

    std::vector<uint32_t> samplerates;
    samplerates.resize(numRates);

    ret = airspyhf_get_samplerates(dev_, samplerates.data(), numRates);
    if (ret != AIRSPYHF_SUCCESS) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_get_samplerates() failed: %d", ret);
        return results;
    }

    // Sort to make short they are in order
    std::sort(samplerates.begin(), samplerates.end());

    for (const auto& samplerate: samplerates) {
        results.push_back(samplerate);
    }

    return results;
}

void SoapyAirspyHF::setBandwidth(const int direction, const size_t channel, const double bw) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "setBandwidth(%d, %d, %f) not supported.", direction, channel, bw);
}

double SoapyAirspyHF::getBandwidth(const int direction, const size_t channel) const
{
    return 0.9 * sampleRate_;
}

std::vector<double> SoapyAirspyHF::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> results;

    for(const auto& samplerate: listSampleRates(direction, channel)) {
        // Reasoable estimate
        // TODO: maybe cache sample rates?
        results.push_back(0.9 * samplerate);
    }

    return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyAirspyHF::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    // Enable DSP in library. TODO. make this optional
    SoapySDR::ArgInfo enableDSPArg;
    enableDSPArg.key = "dsp";
    enableDSPArg.value = "true";
    enableDSPArg.name = "DSP";
    enableDSPArg.description = "Enable DSP";
    enableDSPArg.type = SoapySDR::ArgInfo::BOOL;

    return setArgs;
}

void SoapyAirspyHF::writeSetting(const std::string &key, const std::string &value)
{
    int ret;

    if(key == "dsp") {
        bool enable = (value == "true");
        // Enables/Disables the IQ Correction, IF shift and Fine Tuning.
        ret = airspyhf_set_lib_dsp(dev_, enable);
        if(ret != AIRSPYHF_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspyhf_set_lib_dsp() failed: (%d)", ret);
        } else {
            SoapySDR::logf(SOAPY_SDR_DEBUG, "airspyhf_set_lib_dsp(%d)", enable);
        }
    }
    else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "writeSetting(%s, %s) not supported.", key.c_str(), value.c_str());
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "writeSetting(%s, %s)", key.c_str(), value.c_str());
}

std::string SoapyAirspyHF::readSetting(const std::string &key) const
{
    if(key == "dsp") {
        return enableDSP_ ? "true" : "false";
    }
    else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "readSetting(%s) not supported.", key.c_str());
        return "";
    }
}
