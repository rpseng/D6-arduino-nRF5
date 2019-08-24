# Arduino Core for Nordic Semiconductor nRF5 based boards


## Supported boards

### nRF52
 * [Plain nRF52 MCU](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52832)
 * [Nordic Semiconductor nRF52 DK](https://www.nordicsemi.com/eng/Products/Bluetooth-Smart-Bluetooth-low-energy/nRF52-DK)
   * For boards prior to ```2016.9``` (see sticker), the lastest JLink bootloader is required to upload sketches. To upgrade, press the boot/reset button while powering on the board and copy over the latest [bootloader](https://www.nordicsemi.com/eng/nordic/Products/nRF52-DK/nRF5x-OB-JLink-IF/52275).
   * To see how the silkscreened MCU pin names map to Arduino-compatible pin names, see ```PCA10040_Schematic_And_PCB.pdf``` -> ```GPIO pin mapping``` from the [PCA10040_Schematic](https://www.nordicsemi.com/eng/nordic/Products/nRF52-DK/nRF52-HW/50980).
 * [Shenzhen Taida Century Technology nRF52 low cost development board](https://www.aliexpress.com/item/NRF52832-high-cost-development-board-gold-core-board/32725601299.html)
 * [RedBear Blend 2](https://github.com/redbear/nRF5x#blend-2)
 * [RedBear Nano 2](https://github.com/redbear/nRF5x#ble-nano-2)
 * [Bluey](https://github.com/electronut/ElectronutLabs-bluey)
 * [hackaBLE](https://github.com/electronut/ElectronutLabs-hackaBLE)
 * [hackaBLE_v2](https://github.com/electronut/ElectronutLabs-hackaBLE)
 * [DWM1001-DEV](https://www.decawave.com/product/dwm1001-development-board/)

### nRF51
 * [Plain nRF51 MCU](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF51822)
 * [BBC micro:bit](https://microbit.org)
 * [Calliope mini](https://calliope.cc/en)
 * [Bluz DK](http://bluz.io)
 * Nordic Semiconductor  [nRF51822 Development Kit](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF51822-Development-Kit) + [nRF51422 Development Kit](https://www.nordicsemi.com/eng/Products/ANT/nRF51422-Development-Kit)
 * Nordic SemiconductornRF51x22 Development Kits (PCA1000x)
 * [Nordic Semiconductor NRF51 Smart Beacon Kit](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF51822-Bluetooth-Smart-Beacon-Kit)
 * [Nordic Semiconductor NRF51 Dongle](http://www.nordicsemi.com/eng/Products/nRF51-Dongle)
 * [OSHChip](http://www.oshchip.org/)
 * [RedBearLab BLE Nano](http://redbearlab.com/blenano/)
 * [RedBearLab nRF51822](http://redbearlab.com/redbearlab-nrf51822/)
 * [Waveshare BLE400](http://www.waveshare.com/wiki/BLE400)
 * [ng-beacon](https://github.com/urish/ng-beacon)
 * [TinyBLE](https://www.seeedstudio.com/Seeed-Tiny-BLE-BLE-%2B-6DOF-Mbed-Platform-p-2268.html)
 * [Sino:bit](http://sinobit.org)
 * [SeeedArchLink](http://wiki.seeedstudio.com/Arch_Link/)

## Installing

### Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (At least v1.6.12)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://sandeepmistry.github.io/arduino-nRF5/package_nRF5_boards_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "Nordic Semiconductor nRF5 Boards"
 6. Select your nRF5 board from the Tools -> Board menu

__NOTE:__ During installation it takes the Arduino IDE a few minutes to extract the tools after they have been downloaded, please be patient.

###### Driver Setup for St-Link

 1. Download [Zadig](http://zadig.akeo.ie)
 2. Plugin St-Link board
 3. Start ```Zadig```
 4. Select ```Options -> List All Devices```
 5. Plug and unplug your device to find what changes, and select the ```St-Link``` from the device dropdown
 6. Click ```Replace Driver```

__NOTE__: To roll back to the original driver go to: Device Manager -> Right click on device -> Check box for "Delete the driver software for this device" and click Uninstall

## Credits

This core is based on the [Arduino SAMD Core](https://github.com/arduino/ArduinoCore-samd) and licensed under the same [LGPL License](LICENSE)

The following tools are used:

 * [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded) as the compiler
 * A [forked](https://github.com/sandeepmistry/openocd-code-nrf5) version of [OpenOCD](http://openocd.org) to flash sketches
