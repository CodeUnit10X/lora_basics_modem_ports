# Introduction

I have several Waveshare (https://www.waveshare.com/product/iot-communication/long-range-wireless/nb-iot-lora.htm) LoRa Hats lying around.  The Software available for them
at the time I wrote this was all based on older Semtech software libraries.

Semtech has since moved to:

See:  https://github.com/Lora-net/SWL2001


This is a port for Pico and Raspberry Pi Waveshare Hats listed below, in the Hardware section.  I did not implement FOUTA or STORE and Forward.  But basical LoRaWan
OTAA and Uplink/Status has been tested.


There are a couple of basic examples, one OTAA and a ping pong for general radio RX/TX based directly off the Semtech Example.

## otaa_example

This demonstrates the basics of using the smtc_modem api to connect to a IOT stack using OTAA.  It then does periodic uplinks of the boards temp.

## ping_pong

This is basic modem RX and TX between two modems.

# Hardware

The following hardware was used for testing and development

## Waveshare LoRaWan Gateway for Raspberry Pi

https://www.waveshare.com/wiki/SX1302_LoRaWAN_Gateway_HAT

## Waveshare Lora Node for Raspberry Pi Pico

https://www.waveshare.com/wiki/Pico-LoRa-SX1262

https://www.waveshare.com/wiki/RP2040-LoRa


## Waveshare Lora Node for Raspberry Pi

https://www.waveshare.com/wiki/SX1262_XXXM_LoRaWAN/GNSS_HAT


# Building:

I build on Arch Linux but you should have no problems on most Linux Distros, you'll need at minimum cmake, and the build environment for your distro (build-essentials etc).

**Currently only the sx1262 radio is supported.**

**set your region correctly for your country, in the top level CMakeLists.txt i.e. -DRADIO_REGION=US_915**

You configure the target board via the following:

PLATFORM_BOARD [ PICO | PICO2 | LINUX ]

cmake -DPLATFORM_BOARD="PICO" -DPICO_SDK_PATH=/usr/share/pico-sdk -DRADIO_REGION=US_915 -DCMAKE_BUILD_TYPE=Release ..

The LINUX Platform is a purely userspace implementation.  It relies on the UIO driver to be configured in your kernel. 

CONFIG_UIO=y
CONFIG_UIO_PDRV_GENIRQ=y

For a complete image to support this with the 
Waveshare Raspberry Pi GNSS_HAT see:

https://github.com/CodeUnit10X/animal-farm

The configuration for Raspberry Pi Zero 2w, includes all the required prereqs.  To create everything in Buildroot for Pi running Linux you
can do the following:

<ol>
	<li>git clone https://github.com/buildroot/buildroot</li>
	<li>git clone https://github.com/CodeUnit10X/animal-farm</li>
	<li>cd <path/to/buildroot> && make BR2_EXTERNAL=</path/to/animal-farm> raspberrypi_2w_custom_defconfig</li>
	<li>make menuconfig External options -> pi application options -> [*] lorawan </li>
	<li>make</li>
</ol>

The initial build of buildroot can take a long time, but once its down you'll have a an sdcard.img in output/images folder of buildroot, that you use
on a pi zero 2w with the Waveshare hat.  You'd need to update for your Region and Dev/app eui's etc.  

If you want to support another Pi, Take a look at   

https://github.com/CodeUnit10X/animal-farm/blob/main/board/raspberrypi/raspberry_pi_zero_2w/cmdline.txt

https://github.com/CodeUnit10X/animal-farm/blob/main/board/raspberrypi/raspberry_pi_zero_2w/config.txt


To build examples standalone (outside Buildroot):

cmake -DPLATFORM_BOARD="LINUX" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=<path to the toolchain file used by buildroot> -DRADIO_REGION=US_915 ..

**note the Linux port is entirely userspace, and thus running on a general purpose OS.  While it seems to work fine on my test system keep in mind your mileage may vary depending on what else is running on your system, as LoRaWan RX timing is pretty precise.**


# System Setup

## Gateway setup

I used Chirpstack v4.6.0 on my Waveshare gateway.  The Waveshare hat was connected to a Pi 4B.  I installed the stock openWRT images on an sdcard.  Chirp stack is well documented, start at the below link:

https://www.chirpstack.io/docs/chirpstack-gateway-os/install/raspberry-pi.html

Create a device profile.

![image](docs/images/chirp1.png)

Create an appliction and add a device.

![image](docs/images/chirp2.png)

![image](docs/images/chirp3.png)

![image](docs/images/chirp4.png)


**note:  Join times can be a bit slow, I believe with my configuration the concentrator was only scanning the first bank of channels 0-7 and the Lora Basics Modem library will go through the entire channel list 0-64.  I did not see any way to mask channel list in Lora Basics Modem API.    


## TODO

- Add GPS support for GNSS HAT
- New Modem test application using the smtc_modem_test stuff