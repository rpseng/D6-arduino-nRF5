# D6 Fitness Tracker Arduino Core for Nordic Semiconductor nRF5 based boards

This core is edited for the D6 Fitness Tracker, it is original from here https://github.com/sandeepmistry/arduino-nRF5

This is the Installable version of the Portable version shown in this Video:
(Click on the Picture to watch the Video)
[![YoutubeVideo](https://img.youtube.com/vi/3gjmEdEDJ5A/0.jpg)](https://www.youtube.com/watch?v=3gjmEdEDJ5A)

After installation the Core works as in the Video

Here is the [D6Flasher](https://play.google.com/store/apps/details?id=com.atcnetz.ble.readwrite) on Google Play-store
with the Flasher it is Possible to flash the Created firmware over the air, please view the Youtube Video.

You can buy the Tracker on Gearbest for as Cheap as 5$ [https://www.gearbest.com/smart-watches/pp_1232618.html?wid=1433363&lkid=52058797](https://www.gearbest.com/smart-watches/pp_1232618.html?wid=1433363&lkid=52058797)
Coupon: GB0722MPOW

## Installing

### Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (At least v1.6.12)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://atc1441.github.io/D6Library/package_nRF5_boards_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "D6 Tracker by ATC1441"
 6. Select the DSD6 Tracker board from the Tools -> Board menu
 7. You will find the DFU update file in a folder like this on windows. ```C:\Users\USERNAME\AppData\Local\Temp\arduino_build_RANDOM\SKETCHNAME.ino.zip```

__NOTE:__ During installation it takes the Arduino IDE a few minutes to extract the tools after they have been downloaded, please be patient.


### Adafruit's nrfutil tools

[adafruit-nrfutil](https://github.com/adafruit/Adafruit_nRF52_nrfutil) (derived from Nordic pc-nrfutil) is needed to create the OTA update file.

- For Windows and macOS, pre-built executable binaries are included in the BSP at `tools/adafruit-nrfutil/`. It should work out of the box.
- Linux users need to run the follow commands to install the nrfutil tools:

    ```
    $ pip3 install wheel --user
    $ pip3 install adafruit-nrfutil --user
	```
    Then make sure `adafruit-nrfutil` executable is in your path, if not try to add the following to your `.bashrc` file:
    ```
    PATH="$HOME/.local/bin/:$PATH"
    ```
    After that you may need to restart your pc.

### Driver Setup for St-Link

 1. Download [Zadig](http://zadig.akeo.ie)
 2. Plugin St-Link board
 3. Start ```Zadig```
 4. Select ```Options -> List All Devices```
 5. Plug and unplug your device to find what changes, and select the ```St-Link``` from the device dropdown
 6. Click ```Replace Driver``` / ```Install Driver```

__NOTE__: To roll back to the original driver go to: Device Manager -> Right click on device -> Check box for "Delete the driver software for this device" and click Uninstall

### Pinout, Flashfiles and Espruino Version

Here [Fanoush](https://github.com/fanoush/ds-d6) made much work in the Espruino direction with the Pinout and Flash Backup files.

## Credits

Copy from the [Sandeepmistry core](https://github.com/sandeepmistry/arduino-nRF5) and edited to make D6 Fitness Tracker Compatible.

This core is based on the [Arduino SAMD Core](https://github.com/arduino/ArduinoCore-samd) and licensed under the same [LGPL License](LICENSE)

Nrfutil taken from [Adafruit nRF52 Library](https://github.com/adafruit/Adafruit_nRF52_Arduino)

The following tools are used:

 * [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded) as the compiler
 * A [forked](https://github.com/sandeepmistry/openocd-code-nrf5) version of [OpenOCD](http://openocd.org) to flash sketches
