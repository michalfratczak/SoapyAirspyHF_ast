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

#include <SoapySDR/ConverterRegistry.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>

#include <algorithm>
#include <libairspyhf/airspyhf.h>
#include <memory>

#define AIRSPYHF_NATIVE_FORMAT SOAPY_SDR_CF32

std::vector<std::string>
SoapyAirspyHF::getStreamFormats(const int direction,
                                const size_t channel) const {
  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "SoapyAirspyHF::getStreamFormats(%d, %d) invalid channel",
                   direction, channel);
    return {};
  }

  // Allow all formats we can convert to
  return SoapySDR::ConverterRegistry::listTargetFormats(AIRSPYHF_NATIVE_FORMAT);
}

std::string SoapyAirspyHF::getNativeStreamFormat(const int direction,
                                                 const size_t channel,
                                                 double &fullScale) const {

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(
        SOAPY_SDR_ERROR,
        "SoapyAirspyHF::getNativeStreamFormat(%d, %d) invalid channel",
        direction, channel);
    return {};
  }

  fullScale = 1.0;
  return AIRSPYHF_NATIVE_FORMAT;
}

SoapySDR::ArgInfoList
SoapyAirspyHF::getStreamArgsInfo(const int direction,
                                 const size_t channel) const {

  SoapySDR::ArgInfoList streamArgs;

  if (direction != SOAPY_SDR_RX or channel != 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "SoapyAirspyHF::getStreamArgsInfo(%d, %d) invalid channel",
                   direction, channel);
    return streamArgs;
  }

  return streamArgs;
}

// Static trampoline for libairspyhf callback
static int rx_callback_(airspyhf_transfer_t *transfer) {
  // Stream handle
  SoapySDR::Stream *stream = (SoapySDR::Stream *)transfer->ctx;

  const uint32_t timeout_us = 500'000; // 500ms

  const auto written = stream->ringbuffer().write_at_least(
      static_cast<size_t>(transfer->sample_count),
      std::chrono::microseconds(timeout_us),
      [&](airspyhf_complex_float_t *begin,
          [[maybe_unused]] const uint32_t available) {
        // Copy samples to ringbuffer, conversion is done in readStream if
        // needed.
        std::copy(transfer->samples, transfer->samples + transfer->sample_count,
                  begin);

        return transfer->sample_count;
      });

  // Add ticks
  stream->addTicks(transfer->sample_count);

  if (written < 0) {
    SoapySDR::logf(SOAPY_SDR_INFO,
                   "SoapyAirspyHF::rx_callback: ringbuffer write timeout");
    return 0;
  }

  return 0; // anything else is an error.
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *
SoapyAirspyHF::setupStream(const int direction, const std::string &format,
                           const std::vector<size_t> &channels,
                           const SoapySDR::Kwargs &args) {

  (void)args; // Currently unused

  if (direction != SOAPY_SDR_RX or channels.size() != 1 or
      channels.at(0) != 0) {
    SoapySDR::logf(SOAPY_SDR_INFO,
                   "SoapyAirspyHF::setupStream(%d, %s, %d, %d) invalid channel",
                   direction, format.c_str(), channels.size(), channels.at(0));
  }

  const auto &sources = SoapySDR::ConverterRegistry::listSourceFormats(format);

  // Check there is a convert function that can convert from our native format.
  if (std::find(sources.begin(), sources.end(), AIRSPYHF_NATIVE_FORMAT) ==
      sources.end()) {
    throw std::runtime_error("setupStream invalid format '" + format + "'.");
  }

  // Find converter function
  const auto converterFunction = SoapySDR::ConverterRegistry::getFunction(
      AIRSPYHF_NATIVE_FORMAT, format, SoapySDR::ConverterRegistry::GENERIC);

  SoapySDR::logf(SOAPY_SDR_INFO, "setupStream: format=%s", format.c_str());

  // Get MTU
  const auto mtu = airspyhf_get_output_size(device_);

  // Create stream;
  stream_ = std::make_unique<SoapySDR::Stream>(device_, sampleRate_,
                                               converterFunction, mtu);

  // Return point to stream
  return stream_.get();
}

void SoapyAirspyHF::closeStream(SoapySDR::Stream *stream) {
  // Check that stream is current
  if (stream != stream_.get()) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "closeStream: invalid stream");
    return;
  }

  stream_.reset();
}

size_t SoapyAirspyHF::getStreamMTU(SoapySDR::Stream *stream) const {
  return stream->MTU();
}

int SoapyAirspyHF::activateStream(SoapySDR::Stream *stream, const int flags,
                                  const long long timeNs,
                                  const size_t numElems) {

  int ret = 0;

  // rate limit this log?
  if (flags != 0) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "activateStream: flags not supported");
  }

  // Clear buffer
  stream->ringbuffer().clear();
  // Reset ticks
  stream->ticks_ = 0;

  // Start the stream
  ret = airspyhf_start(device_, &rx_callback_, (void *)stream);
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "activateStream: airspyhf_start failed: %d",
                   ret);
    return SOAPY_SDR_STREAM_ERROR;
  }

  SoapySDR::logf(SOAPY_SDR_DEBUG,
                 "activateStream: flags=%d, timeNs=%lld, numElems=%d", flags,
                 timeNs, numElems);

  return 0;
}

int SoapyAirspyHF::deactivateStream(SoapySDR::Stream *stream, const int flags,
                                    const long long timeNs) {
  int ret = 0;

  SoapySDR::logf(SOAPY_SDR_DEBUG, "deactivateStream: flags=%d, timeNs=%lld",
                 flags, timeNs);

  if (flags != 0) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "deactivateStream: flags not supported");
  }

  // Stop streaming
  ret = airspyhf_stop(stream->device());
  if (ret != AIRSPYHF_SUCCESS) {
    SoapySDR::logf(SOAPY_SDR_ERROR,
                   "deactivateStream: airspyhf_stop() failed: %d", ret);
    return SOAPY_SDR_STREAM_ERROR;
  }

  return 0;
}

int SoapyAirspyHF::readStream(SoapySDR::Stream *stream, void *const *buffs,
                              const size_t numElems, int &flags,
                              long long &timeNs, const long timeoutUs) {

  // Flags are not used by this driver
  flags = 0;

  // Convert either requested number of elements or the MTU.
  const auto to_convert = std::min(numElems, getStreamMTU(stream));

  const auto converted = stream->ringbuffer().read_at_least(
      to_convert, std::chrono::microseconds(timeoutUs),
      [&](const airspyhf_complex_float_t *begin,
          [[maybe_unused]] const uint32_t available) {
        // Convert samples to output buffer
        stream->converter()(begin, buffs[0], to_convert, 1.0);

        // Consume from ringbuffer
        return to_convert;
      });

  timeNs = stream->timeNs();

  if (converted < 0) {
    SoapySDR::logf(SOAPY_SDR_INFO, "readStream: ringbuffer read timeout.");
    return SOAPY_SDR_TIMEOUT;
  }

  return static_cast<int>(converted);
}
