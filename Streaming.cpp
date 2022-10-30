/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe
 * Copyright (c) 2018 Corey Stotts

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
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/ConverterRegistry.hpp>
#include <algorithm> //min
#include <climits> //SHRT_MAX
#include <cstring> // memcpy
#include <chrono>


#define SOAPY_NATIVE_FORMAT SOAPY_SDR_CF32

std::vector<std::string> SoapyAirspyHF::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;

    UNUSED(direction);

    for (const auto &target : SoapySDR::ConverterRegistry::listTargetFormats(SOAPY_NATIVE_FORMAT)) {
        formats.push_back(target);
    }

    return formats;
}

std::string SoapyAirspyHF::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
    UNUSED(direction);

    fullScale = 1.0;
    return SOAPY_NATIVE_FORMAT;
}

SoapySDR::ArgInfoList SoapyAirspyHF::getStreamArgsInfo(const int direction, const size_t channel) const {
    SoapySDR::ArgInfoList streamArgs;

    // TODO
    // SoapySDR::ArgInfo chanArg;
    // chanArg.key = "chan";
    // chanArg.value = "mono_l";
    // chanArg.name = "Channel Setup";
    // chanArg.description = "Input channel configuration.";
    // chanArg.type = SoapySDR::ArgInfo::STRING;
    // std::vector<std::string> chanOpts;
    // std::vector<std::string> chanOptNames;
    // chanOpts.push_back("mono_l");
    // chanOptNames.push_back("Mono Left");
    // chanOpts.push_back("mono_r");
    // chanOptNames.push_back("Mono Right");
    // chanOpts.push_back("stereo_iq");
    // chanOptNames.push_back("Complex L/R = I/Q");
    // chanOpts.push_back("stereo_qi");
    // chanOptNames.push_back("Complex L/R = Q/I");
    // chanArg.options = chanOpts;
    // chanArg.optionNames = chanOptNames;
    // streamArgs.push_back(chanArg);

    return streamArgs;
}

/****************************************g***************************
 * Async thread work
 ******************************************************************/

// Static trampoline for libairspyhf callback
static int _rx_callback(airspyhf_transfer_t *transfer)
{
    SoapyAirspyHF *self = (SoapyAirspyHF *)transfer->ctx;
    return self->rx_callback(transfer);
}

int SoapyAirspyHF::rx_callback(airspyhf_transfer_t *transfer)
{

    // Wait for a buffer to become ready
    std::unique_lock<std::mutex> lock(bufferLock_);

    // TODO: maybe timeout here?
    auto const timeout = std::chrono::milliseconds(1000);
    const bool wait_ret = bufferReady_.wait_for(lock, timeout, [&] {
        return bufferPtr_ != nullptr ||
            !airspyhf_is_streaming(dev_);
    });

    // Did we time out?
    if(wait_ret == false) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "rx_callback: timeout waiting for buffer.");
        return -1;
    }

    // If not streaming, we are done.
    if(!airspyhf_is_streaming(dev_)) {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_callback: not streaming");
        return -1;
    }

    // Print a warning if we have dropped samples.
    if(transfer->dropped_samples > 0) {
        SoapySDR::logf(SOAPY_SDR_WARNING, "Dropped %d samples",
                       transfer->dropped_samples);
    }

    // Convert the samples directly into the buffer.
    converterFunction_(transfer->samples, bufferPtr_, transfer->sample_count, 1.0);

    // Ok we are done, invalidate the buffer pointer and notify
    bufferPtr_ = nullptr;
    callbackDone_.notify_one();

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
    UNUSED(direction);

    //check the channel configuration
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
    UNUSED(stream);
}

size_t SoapyAirspyHF::getStreamMTU(SoapySDR::Stream *stream) const {
    // This value is a constant in the driver
    return airspyhf_get_output_size(dev_);
}

int SoapyAirspyHF::activateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs,
        const size_t numElems)
{
    int ret;
    // No flags supported
    if (flags != 0) { return SOAPY_SDR_NOT_SUPPORTED; }
    // Start the stream
    ret = airspyhf_start(dev_, &_rx_callback, (void *)this);

    if (ret != AIRSPYHF_SUCCESS) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    SoapySDR::logf(SOAPY_SDR_DEBUG, "activateStream: flags=%d, timeNs=%lld, numElems=%d", flags, timeNs, numElems);

    return 0;
}

int SoapyAirspyHF::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    int ret;
    // No flags supported
    if (flags != 0) { return SOAPY_SDR_NOT_SUPPORTED; }

    // Stop streaming
    ret = airspyhf_stop(dev_);

    if (ret != AIRSPYHF_SUCCESS) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    // Don't hang in callback
    bufferReady_.notify_one();
    callbackDone_.notify_one();

    return 0;
}

int SoapyAirspyHF::readStream(SoapySDR::Stream *stream,
                              void * const *buffs,
                              const size_t numElems,
                              int &flags,
                              long long &timeNs,
                              const long timeoutUs) {

    if(flags != 0) { return SOAPY_SDR_NOT_SUPPORTED; }

    // We needs at least this many elements
    if(numElems < getStreamMTU(stream)) {
        SoapySDR_logf(SOAPY_SDR_WARNING, "readStream numElems=%d < MTU=%d",
                      numElems, getStreamMTU(stream));
        return 0;
    }

    std::unique_lock<std::mutex> lock(bufferLock_);
    // Store pointer to buffer
    bufferPtr_ = buffs[0];
    // We are ready for the callback to fill the buffer
    bufferReady_.notify_one();

    // The callback will set this pointer to nullptr when it is done.
    const auto timeout = std::chrono::microseconds(timeoutUs);
    const bool wait_ret = callbackDone_.wait_for(lock, timeout, [&] {
        return bufferPtr_ == nullptr ||    // Callback is done
            !airspyhf_is_streaming(dev_); // We are not streaming anymore
    });

    // Did we time out?
    if(wait_ret == false) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "readStream: timeout waiting for callback.");
        return SOAPY_SDR_TIMEOUT;
    }

    // Check condiation, if we are not streaming anymore, we are done.
    // Otherwise, it means data was written to the buffer.
    if (!airspyhf_is_streaming(dev_)) {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "readStream: not streaming");
        // TODO.
        return SOAPY_SDR_STREAM_ERROR;
    }

    // SoapySDR_logf(SOAPY_SDR_DEBUG, "readStream: %d", numElems);

    // Only work with the fastest block size
    return getStreamMTU(stream);
}
