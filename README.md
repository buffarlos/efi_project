# Teensy EFI Project

The goal of this project is to implement a fuel injection system based on the Teensy 4.0 microcontroller. The firmware is designed to read engine speed and MAP in order to determine fuel injection load based on a lookup VE table.

## Installation

### Software

efi_project/efi/efi.ino is the source code for the Teensy EFI project, and is intended to be compiled and flashed onto a Teensy 4.0 unit with the Arduino IDE. Follow this link for instructions on how to use Teensy with the Arduino IDE: https://www.pjrc.com/teensy/td_download.html

### Hardware

The firmware is written for Teensy 4.0. Teensy 4.0 should be configured to have an injector signal digital output pin, a Hall effect switch digital input pin, and a MAP sensor analog input pin. The injector driver hardware should take a square wave signal from the injector signal pin in order to time injector opening. The Hall effect switch pin will trigger an interrupt in the firmware to detect trigger wheel tooth passage. The interrupt trigger is FALLING to accommodate the originally used sensor and trigger wheel, but should be adjusted as needed to the specific application. Likewise, MAP sensor calibration will need to be performed for each application.