# Teensy EFI Project

The goal of this project is to implement a fuel injection system based on the Teensy 4.0 microcontroller. The firmware is designed to read engine speed and MAP in order to determine fuel injection load based on a lookup VE table.

## Installation

### Software

efi_project/efi/efi.ino is the source code for the Teensy EFI project, and is intended to be compiled and flashed onto a Teensy 4.0 unit with the Arduino IDE. Follow this link for instructions on how to use Teensy with the Arduino IDE: https://www.pjrc.com/teensy/td_download.html

efi_project/test_tooth_detection/test_tooth_detection.ino is source code for a Hall effect switch function test. It generates a digital output to the specified GPIO pin in response to rising or falling Hall switch signal. Use this script to check Hall switch function.

efi_project/test_injector/test_injector.ino is source code for an injector function test script. It takes number of injections, injection length, and simulated engine speed as inputs over serial communications and fires the injector according to the specified parameters. This is useful for testing injector function and metering injection flow.

### Hardware

The firmware is written for Teensy 4.0. Teensy 4.0 should be configured to have an injector signal digital output pin, a Hall effect switch analog input pin, and a MAP sensor analog input pin. The injector driver hardware should take a square wave signal from the injector signal pin in order to time injector opening (such as LM1949 from Texas Instruments). The Hall effect switch pin will detect trigger wheel tooth passage when pin voltage falls below a set threshold. A tooth is detected when voltage falls below the threshold to accommodate the originally used sensor and trigger wheel, but should be adjusted as needed to the specific application. Likewise, MAP sensor calibration will need to be performed for each application.