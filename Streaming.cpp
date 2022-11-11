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
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/ConverterRegistry.hpp>
#include <algorithm>
#include <climits>
#include <cstring>
#include <chrono>


#define SOAPY_NATIVE_FORMAT SOAPY_SDR_CF32


std::vector<std::string> SoapyAirspyHF::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;

    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyAirspyHF::getStreamFormats(%d, %d) invalid channel", direction, channel);
        return formats;
    }

    for (const auto &target : SoapySDR::ConverterRegistry::listTargetFormats(SOAPY_NATIVE_FORMAT)) {
        formats.push_back(target);
    }

    return formats;
}

std::string SoapyAirspyHF::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {

    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyAirspyHF::getNativeStreamFormat(%d, %d) invalid channel", direction, channel);
        return "";
    }

    fullScale = 1.0;
    return SOAPY_NATIVE_FORMAT;
}

SoapySDR::ArgInfoList SoapyAirspyHF::getStreamArgsInfo(const int direction, const size_t channel) const {

    SoapySDR::ArgInfoList streamArgs;

    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyAirspyHF::getStreamArgsInfo(%d, %d) invalid channel", direction, channel);
        return streamArgs;
    }

    return streamArgs;
}

/*******************************************************************
 * Async thread work
 ******************************************************************/

// Static trampoline for libairspyhf callback
static int rx_callback_(airspyhf_transfer_t *transfer)
{
    SoapyAirspyHF *self = (SoapyAirspyHF *)transfer->ctx;
    return self->rx_callback(transfer);
}

int SoapyAirspyHF::rx_callback(airspyhf_transfer_t *transfer)
{

    const uint32_t timeout_us = 500000; // 500ms

    const auto written = ringbuffer_.write_at_least
        (transfer->sample_count,
         std::chrono::microseconds(timeout_us),
         [&](airspyhf_complex_float_t* begin, [[maybe_unused]] const uint32_t available) {
             // Copy samples to ringbufer
             std::copy(transfer->samples,
                       transfer->samples + transfer->sample_count,
                       begin);

             return transfer->sample_count;
         });

    // Count ticks
    ticks_.fetch_add(transfer->sample_count, std::memory_order_release);

    if(written < 0) {
        SoapySDR::logf(SOAPY_SDR_INFO, "SoapyAirspyHF::rx_callback: ringbuffer write timeout");
        return 0;
    }

    return 0; // anything else is an error.
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapyAirspyHF::setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs &args)
{
    if(direction != SOAPY_SDR_RX or channels.size() != 1 or channels.at(0) != 0) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setupStream: RX not supported");
        return nullptr;
    }

    // Check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0)) {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    std::vector<std::string> sources = SoapySDR::ConverterRegistry::listSourceFormats(format);

    if (std::find(sources.begin(), sources.end(), SOAPY_NATIVE_FORMAT) == sources.end()) {
        throw std::runtime_error(
                "setupStream invalid format '" + format + "'.");
    }

    // Find converter functinon
    converterFunction_ = SoapySDR::ConverterRegistry::getFunction(SOAPY_NATIVE_FORMAT, format, SoapySDR::ConverterRegistry::GENERIC);

    SoapySDR::logf(SOAPY_SDR_INFO, "setupStream: format=%s", format.c_str());

    return (SoapySDR::Stream*) this;
}

void SoapyAirspyHF::closeStream(SoapySDR::Stream *stream)
{
    (void) stream;
}

size_t SoapyAirspyHF::getStreamMTU(SoapySDR::Stream *stream) const {
    // This value is a constant in the driver
    return airspyhf_get_output_size(dev_);
}

int SoapyAirspyHF::activateStream(SoapySDR::Stream *stream,
                                  const int flags,
                                  const long long timeNs,
                                  const size_t numElems)
{
    int ret;
    // No flags supported
    if (flags != 0) {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "activateStream: flags not supported");
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    // Clear ringbuffer
    ringbuffer_.clear();

    // Reset ticks
    ticks_ = 0;

    // Start the stream
    ret = airspyhf_start(dev_, &rx_callback_, (void *)this);
    if (ret != AIRSPYHF_SUCCESS) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    SoapySDR::logf(SOAPY_SDR_DEBUG, "activateStream: flags=%d, timeNs=%lld, numElems=%d", flags, timeNs, numElems);

    return 0;
}

int SoapyAirspyHF::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    int ret;

    SoapySDR::logf(SOAPY_SDR_DEBUG, "deactivateStream: flags=%d, timeNs=%lld", flags, timeNs);

    // No flags supported
    if (flags != 0) {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "deactivateStream: flags not supported");
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    // Stop streaming
    ret = airspyhf_stop(dev_);

    if (ret != AIRSPYHF_SUCCESS) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    return 0;
}

int SoapyAirspyHF::readStream(SoapySDR::Stream *stream,
                              void * const *buffs,
                              const size_t numElems,
                              int &flags,
                              long long &timeNs,
                              const long timeoutUs) {

    if(flags != 0) {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "readStream: flags not supported");
    }

    // Convert either requested number of elements or the MTU.
    const auto to_convert = std::min(numElems, getStreamMTU(stream));

    const auto converted = ringbuffer_.read_at_least
        (to_convert,
         std::chrono::microseconds(timeoutUs),
         [&](const airspyhf_complex_float_t* begin, [[maybe_unused]] const uint32_t available) {
             // Convert samples to output buffer
             converterFunction_(begin,
                                buffs[0],
                                to_convert,
                                1.0);

             // Consume from ringbuffer
             return to_convert;
         });

    // Count ticks to return time
    timeNs = SoapySDR::ticksToTimeNs(ticks_.load(std::memory_order_acquire),
                                     sampleRate_);

    if(converted < 0) {
        SoapySDR::logf(SOAPY_SDR_INFO, "readStream: ringbuffer read timeout.");
        return SOAPY_SDR_TIMEOUT;
    }

    return converted;
}
