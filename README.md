# ublox_modm_example

This repository contains a rudimentary modm c++ example driver for the [ZED-f9P](https://www.u-blox.com/en/product/zed-f9p-module?lang=de). 

## How can I configure the ublox ZED-F9P?

* https://github.com/DroidDrive/ublox_gnss_testbench

## How do I use it?

* git submoduel update --init --recursive
* lbuild build
* scons
* connect nucleo-f439zi developer board
* connect ublox ZED-F9P 
* scons program

## How do I connect the nucleo board?

Connect the following things

* GNSS Antenna on the ZED-F9P
* VCC 5V between ZED-F9P & Nucelo
* VCC 3.3V between ZED-F9P & Nucelo
* GND between ZED-F9P & Nucelo
* UART between ZED-F9P & Nucelo
  * Zed-F9P[RX/MOSI] <---> NucleoF439Zi[CP9, Pin D53 - which is connected to STM32 Pin PD5] [[see here]](https://www.st.com/resource/en/user_manual/dm00244518-stm32-nucleo144-boards-mb1137-stmicroelectronics.pdf)
  * Zed-F9P[TX/MISO] <---> NucleoF439Zi[CP9, Pin D52 - which is connected to STM32 Pin PD6] [[see here]](https://www.st.com/resource/en/user_manual/dm00244518-stm32-nucleo144-boards-mb1137-stmicroelectronics.pdf)

![signal-2021-09-26-035507](https://user-images.githubusercontent.com/6985609/134790522-273adc4a-45ad-4829-bf65-e784d7de6ffb.jpeg)

## Some more important things?

* the ZED-F9P's default configuration only allows the device to send `NMEA` messaged via UART1 with a baudrate of 38400
* the ZED-F9P's default configuration only allows the device to send `NMEA` messaged via USB with a baudrate of 9600 (which is used by the u-center tool [in our testbench](https://github.com/DroidDrive/ublox_gnss_testbench))
* the driver is waiting for `UBX`messages, which are not configured to be outputted by the ZED-F9P, so one has to configure the ZED-f9P first via u-center and enable the required `UBX` messages

