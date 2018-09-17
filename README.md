

# moovbittag-arduino

Arduino101 sketch used in MOOVBIT project

## Overview

This sketch is running on the MOOVBIT TAG device in the MOOVBIT project. The MOOVBIT project is an IOT architecture for monitoring athletes in sports. 
The two sketches (one for collecting sensors data and exposing them, the other for TDD tests) are a minor part of a bigger software architecture. Also I freely released [tests of IOT procotols](https://github.com/nicolocarpignoli/iot-protocol-jtesting) and [a library for Blueetoth Low Energy communication](https://github.com/nicolocarpignoli/jgatttool) on other Github repositories.

In `security-fix` branch you can find an implementation of a simple authentication schema (called HSEC) using "Siphash", a HMAC-like library. This allows MOOVBIT TAG device exchange data only with authorized readers/writers.


## Documentation

All informations can be found downloading the thesis document @ http://amslaurea.unibo.it/12570/


## License

This project is released under MIT license as you can check in LICENSE.md file.
