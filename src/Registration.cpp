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
#include <SoapySDR/Registry.hpp>

#include <fmt/core.h>

static std::vector<SoapySDR::Kwargs>
findAirspyHF(const SoapySDR::Kwargs &args) {

  (void)args; // unused (for now)

  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "findAirspyHF");

  std::vector<SoapySDR::Kwargs> results;

  airspyhf_lib_version_t asVersion;
  airspyhf_lib_version(&asVersion);

  SoapySDR::logf(SOAPY_SDR_DEBUG, "AirSpyHF Lib v%d.%d rev %d",
                 asVersion.major_version, asVersion.minor_version,
                 asVersion.revision);

  uint64_t serials[MAX_DEVICES];

  const int count = airspyhf_list_devices(serials, MAX_DEVICES);
  if (count == AIRSPYHF_ERROR) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "libairspyhf error listing devices");
    return results;
  }

  SoapySDR::logf(SOAPY_SDR_DEBUG, "%d AirSpy boards found.", count);

  // Iterate over found serials
  for (int i = 0; i < count; i++) {
    SoapySDR::Kwargs soapyInfo;

    soapyInfo["serial"] = fmt::format("{:016x}", serials[i]);
    soapyInfo["label"] = fmt::format("AirSpy HF+ [{}]", soapyInfo["serial"]);

    SoapySDR::logf(SOAPY_SDR_DEBUG, "Found device %s",
                   soapyInfo.at("label").c_str());

    results.push_back(soapyInfo);
  }

  return results;
}

static SoapySDR::Device *makeAirspyHF(const SoapySDR::Kwargs &args) {
  // Log debug
  SoapySDR::logf(SOAPY_SDR_DEBUG, "makeAirspyHF");

  return new SoapyAirspyHF(args);
}

// Register with driver table
static SoapySDR::Registry registerAirspyHF("airspyhf", &findAirspyHF,
                                           &makeAirspyHF,
                                           SOAPY_SDR_ABI_VERSION);
