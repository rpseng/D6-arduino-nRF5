# D6 Fitness Tracker Arduino Core for Nordic Semiconductor nRF5 based boards

This core is edited for the D6 Fitness Tracker, it is original from here https://github.com/sandeepmistry/arduino-nRF5

This is the Installable version of the Portable version shown in this Video:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/3gjmEdEDJ5A/0.jpg)](https://www.youtube.com/watch?v=3gjmEdEDJ5A)

## Installing

### Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (At least v1.6.12)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://atc1441.github.io/D6Library/package_nRF5_boards_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "Nordic Semiconductor nRF5 Boards"
 6. Select your nRF5 board from the Tools -> Board menu

__NOTE:__ During installation it takes the Arduino IDE a few minutes to extract the tools after they have been downloaded, please be patient.

###### Driver Setup for St-Link

 1. Download [Zadig](http://zadig.akeo.ie)
 2. Plugin St-Link board
 3. Start ```Zadig```
 4. Select ```Options -> List All Devices```
 5. Plug and unplug your device to find what changes, and select the ```St-Link``` from the device dropdown
 6. Click ```Replace Driver``` / ```Install Driver```

__NOTE__: To roll back to the original driver go to: Device Manager -> Right click on device -> Check box for "Delete the driver software for this device" and click Uninstall

## Credits

Copy from the [Sandeepmistry core](https://github.com/sandeepmistry/arduino-nRF5) and edited to make D6 Fitness Tracker Compatible.

This core is based on the [Arduino SAMD Core](https://github.com/arduino/ArduinoCore-samd) and licensed under the same [LGPL License](LICENSE)

Nrfutil taken from [Adafruit nRF52 Library](https://github.com/adafruit/Adafruit_nRF52_Arduino)

The following tools are used:

 * [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded) as the compiler
 * A [forked](https://github.com/sandeepmistry/openocd-code-nrf5) version of [OpenOCD](http://openocd.org) to flash sketches
