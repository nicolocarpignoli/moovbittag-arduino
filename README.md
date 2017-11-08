

# moovbittag-arduino
Arduino101 sketch used in MOOVBIT project

## Overview

This sketch is the source code running on the MOOVBIT TAG device in the MOOVBIT project. The MOOVBIT project it's a IoT architecture for monitoring athletes in sports. The two sketches (one for collecting sensors data and exposing them, the other for TDD tests) are a minor part of the software of the architecture; I also freely released on other repo's on GitHub tests about iot procotols and a library for Blueetoth Low Energy communication. 

In `security-fix` branch you can find an implementation of a simple authentication schema (called HSEC) using "Siphash", a HMAC-like library. This allows MOOVBIT TAG device exchange data only with authorized readers/writers .



## Documentation

All info can be found downloading the thesis @ http://amslaurea.unibo.it/12570/


## License

This project is released under MIT license as you can check in LICENSE.md file.
