# Introduction

This is a port of Semtechs Lora Basics Modem API for several Waveshare Raspberry Pi Lora Hats, on pico, pico2 and a user space port for Linux.

See:  https://github.com/Lora-net/SWL2001

# Hardware

The following hardware was used for testing and development

## Waveshare LoRaWan Gateway for Raspberry Pi

https://www.waveshare.com/wiki/SX1302_LoRaWAN_Gateway_HAT

## Waveshare Lora Node for Raspberry Pi Pico

https://www.waveshare.com/wiki/Pico-LoRa-SX1262


## Waveshare Lora Node for Raspberry Pi

https://www.waveshare.com/wiki/SX1262_XXXM_LoRaWAN/GNSS_HAT


# Building:

The build system used cmake on a Linux system.

PLATFORM_BOARD [ PICO | PICO2 | ZERO_2W ]

cmake -DPLATFORM_BOARD="PICO2" -DCMAKE_BUILD_TYPE=Release ..

For non-pico standalone builds (outside buildroot), you'll need to specify a toolchain file

cmake -DPLATFORM_BOARD="ZERO_2W" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=/home/jcoffman/work/buildroot_rpi2w/output/host/share/buildroot/toolchainfile.cmake ..

# System Setup

## Gateway setup

I used Chirpstack v4.6.0 on my Waveshare gateway.  The Waveshare hat was connected to a Pi 4-B.  I installed the stock openWRT images on an sdcard.  Chirp stack is well documented, start at the below link:

https://www.chirpstack.io/docs/chirpstack-gateway-os/install/raspberry-pi.html

Create a device profile.

![image](docs/images/chirp1.png)

Create an appliction and add a device.






