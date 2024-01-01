# Soapy SDR plugin for AirspyHF+

## This branch (v2)

**This branch is work in progress**.

The code is significantly simpler than the previous version and most
likely it has better performance. Quite a few bugs have been fixed,
and some new ones have probably been introduced. Please help me find
them! :-)

I have tested this with GQRX and OpenWebRX with good results.

## Building

Make sure you have SoapySDR and libfmt-dev installed.

To build and also create `compile_commands.json`:

    cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
    make
    sudo make install

## Using

Use this Device string in GQRX:

    soapy=0,driver=airspyhf

## Code style

Code style is llvm. There's a `.clang-format` file checked in.

## Testing

I mostly test with GQRX. Please report if you use it with other SDR
software.

Tested with this version of SoapySDR:

    Lib Version: v0.8.1-gbb33b2d2
    API Version: v0.8.200
    ABI Version: v0.8-3

## TODO

* More testing.
* More linting.
* CI/CD.

## Support me

Please consider sponsoring me on GitHub if you enjoy this
work. Everything I earn through donations will go to families and
children of Ukraine who have lost their homes because of the war.

## Dependencies

* SoapySDR - https://github.com/pothosware/SoapySDR/wiki
* libairspyhf - https://github.com/airspy/airspyhf

## Documentation

* https://github.com/pothosware/SoapyAirspyHF/wiki

Turning AGC on/off, LNA and RF gain settings requires firmware R1.3 or
later and airspyhf 1.1.2 or later.
