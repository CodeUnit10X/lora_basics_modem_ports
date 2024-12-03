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

This is basic modem RX and TX between two modems.  It's heavily based off the orginal Semtech pingpong application in their examples.

To run it you'll need two units of course like so:

![image](docs/images/picos.png)

cp ping_pong.uf2 to each unit, or ping_pong.elf if your using debugger.  You should see terminal output like the following:

![image](docs/images/ping_pong.png)


# Hardware

The following hardware was used for testing and development

## Waveshare LoRaWan Gateway for Raspberry Pi

https://www.waveshare.com/wiki/SX1302_LoRaWAN_Gateway_HAT

## Waveshare Lora Node for Raspberry Pi Pico (Pico HAT for either pico/pico2)

https://www.waveshare.com/wiki/Pico-LoRa-SX1262

## Waveshare RP2040-LoRa

https://www.waveshare.com/wiki/RP2040-LoRa


## Waveshare Lora Node for Raspberry Pi

https://www.waveshare.com/wiki/SX1262_XXXM_LoRaWAN/GNSS_HAT


# Building:

I build on Arch Linux but you should have no problems on most Linux Distros, you'll need at minimum cmake, and the build environment for your distro (build-essentials etc), as well as pico-sdk v2.0 if your going to build the pico versions.    

**only the sx1262 radio is supported, I have no other radios**

**set your region correctly for your country, in the top level CMakeLists.txt i.e. -DRADIO_REGION=US_915**

**make sure you have the appropriate frequencies for your region as well, if you using ping_pong example**

There are essentially two supported HATs and a Standalone board with integrated RP2040 and sx1262.  The pico hats will work with either version of PICO but you must configure the system correctly for your board.  This is done by setting the cmake variable below:

PLATFORM_BOARD [ PICO | PICO2 | RP2040_LORA | LINUX ]


## Build for PICO/PICO2 and RP2040_LORA

I you have one of the above Waveshare PICO/PICO2 Hats or the newer RP2040_LORA you just should need to install the PICO SDK V2.0 and do the following the following:

<ol>
	<li>mkdir build_pico2 && cd build_pico2</li>
	<li>cmake -DPLATFORM_BOARD="PICO2" -DPICO_SDK_PATH=/usr/share/pico-sdk -DRADIO_REGION=US_915 -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON ..</li>
	<li>make -j 24</li>
</ol>

## Build for LINUX

If you have the GNSS LORA HAT, and RPI (Zero 2W, 3, 4B, etc)  you can build for your Linux Environment.  You can build it standalone or in the context of buildroot.  If you have a RPI Zero 2W I have a buildroot configuration already setup (see below).  The LINUX Platform is a purely userspace implementation.  

**Note:  Lora Basics Modem expects a flash device for storing LoRa context and other stuff, in this Linux example we dummy up flash with a mapped file.  So there is just a file flash.bin that gets created.  It will persist but you can nuke it if you want to 
clear things out.**

### Basic Standalone build (Crosscompile)

Pre-reqs:
- cmake 3.22
- gcc 14
- libgpiod-dev 1.6.x
- Kernel configured with UIO and uio_pdrv_genirq, spidev

<ol>
	<li>mkdir build_linux && cd build_linux</li>
	<li>cmake -DPLATFORM_BOARD=LINUX -DRADIO_REGION=US_915 -DCMAKE_TOOLCHAIN_FILE=path/to/your/toolchain.cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON ..</li>	
	<li>make -j 24</li>	
</ol>

### Buildroot Image

For a complete Buildroot configuration to support this with the Waveshare Raspberry Pi GNSS_HAT on a Pi Zero 2W see:

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
on a pi zero 2w with the Waveshare hat.  The examples are located in /usr/bin.  You'd need to update for your Region and Dev/app eui's etc.  Generally
I update the package .mk to point to a local clone of the repo and modify it there.  

If you want to support another Pi (3, 4B, etc), Take a look at how I did it under:

https://github.com/CodeUnit10X/animal-farm/blob/main/board/raspberrypi/raspberry_pi_zero_2w/

You'll need UIO driver support in your kernel.

CONFIG_UIO=y
CONFIG_UIO_PDRV_GENIRQ=y

Also look at the following:

https://github.com/CodeUnit10X/animal-farm/blob/main/board/raspberrypi/raspberry_pi_zero_2w/cmdline.txt

https://github.com/CodeUnit10X/animal-farm/blob/main/board/raspberrypi/raspberry_pi_zero_2w/config.txt

**note the Linux port is entirely userspace, and thus running on a general purpose OS.  While it seems to work fine on my test system keep in mind your mileage may vary depending on what else is running on your system, as LoRaWan RX timing is pretty precise.**

### Building on Target (RPI Image)

If you want to build direcly on you PI here are some basic instructions to get it going.  I tested the Waveshare LoRa GNSS HAT on a RPI 4B.

I only tested this on Ubuntu Server images as they have more up to date compiler.  I used the following image:

![image](docs/images/build2os.png)

Setup the build pre-reqs and git the software.

<ol>
	<li>sudo apt-get install build-essential cmake git libgpiod-dev</li>
	<li>git clone https://github.com/CodeUnit10X/lora_basics_modem_ports.git</li>
	<li>cd lora_basics_modem_ports && mkdir build_linux && cd build_linux</li>
	<li>cmake -DPLATFORM_BOARD=LINUX -DRADIO_REGION=US_915 -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON ..</li>
</ol>

The Semtech LoRa Basics Modem library isn't really setup to build on target, as its geared more for crosscompiling for an MCU.  So there is some tinkering that has to be done with the common.mk file.  A patch could be created for this is someone so desired...


edit:  lora_basics_modem_ports/build_linux/subprojects/Source/lbm_lib/makefiles/common.mk at around line 21, remove $(PREFIX) from AR and SZ

```
ifdef GCC_PATH
AR  = $(GCC_PATH)/$(PREFIX)ar
CC  = $(GCC_PATH)/$(PREFIX)gcc
CPP = $(GCC_PATH)/$(PREFIX)g++
AS  = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP  = $(GCC_PATH)/$(PREFIX)objcopy
SZ  = $(GCC_PATH)/$(PREFIX)size
else
AR = ar
CC = $(PREFIX)gcc
CPP = $(PREFIX)g++
AS = $(PREFIX)g++ -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = size
endif

```

At that point you can build the library and examples with:

> make


You'll need spidev enabled and uio irqs as mentioned previously.  You can read this post which describes how todo it for RPI.

https://superuser.com/questions/1466150/linux-uio-pdrv-genirq-module-does-not-create-device

Essentially just create dts file with the following:

```
/dts-v1/;
/plugin/;

/ {
        fragment@0 {
                target = <&gpio>;
                __overlay__ {
                        mfrc522_uio_pins: mfrc522_uio_pins {
                                brcm,pins = <4>;
                                brcm,function = <0>; //<BCM2835_FSEL_GPIO_IN>;
                                brcm,pull = <0>; //<BCM2835_PUD_OFF>;
                        };
                };
        };

        fragment@1 {
                target-path = "/";
                __overlay__ {
                        mfrc522_uio: mfrc522_uio {
                                compatible = "mfrc522-uio,generic-uio";
                                interrupt-parent = <&gpio>;
                                interrupts = <16 1>; // IRQ_TYPE_EDGE_RISING
                                pinctrl-names = "default";
                                pinctrl-0 = <&mfrc522_uio_pins>;
                                status = "okay";
                        };
                };
        };

        __overrides__ {
                gpiopin = <&mfrc522_uio_pins>,"brcm,pins:0",
                        <&mfrc522_uio>,"interrupts:0";
        };
};

```

Basically we just put the dio interrupt number in there for the Waveshare hat which is GPIO 16 on the PI.

Create an overlay:

dtc -I dts -O dtb mfrc522-uio-overlay.dts > uio_overlay.dtbo

cp uio_overlay.dtbo /boot/firmware/overlays

Create a file in /etc/modprobe.d/uio_pdrv_genirq.conf with:

```
options uio_pdrv_genirq of_id=mfrc522-uio,generic-uio
```

In your /boot/config.txt add the following lines towards the end (I can't recall if I needed both the conf file and dtoverlay entry for uio probably not, anyway...):

```
dtoverlay=uio_overlay
dtoverlay=spi0-1cs,cs0_pin=21
```

You can now reboot and should have:

/dev/uio0
/dev/spidev0.0

You can now run the examples using sudo:

~$ sudo lora_basics_modem_ports/build_linux/examples/lorawan/otaa_example 



## Build Artifacts

Once the build is complete there are several artifacts created for the specified platform:

${CMAKE_BINARY_DIR}/stage/lib

 <ol>
	<li>basic_modem.a                - The LoRa Basics Modem Library</li>
	<li>basic_modem_sx1262_notrace.a - Same as above sans trace</li>
	<li>liblora_basics_modem_hal.a   - The hal and bsp specifics for the platform</li>
</ol>

API includes for top level smtc hal calls etc found in:

${CMAKE_BINARY_DIR}/stage/include 

If you selected CMake option BUILD_EXAMPLES=ON then the examples will be in 

${CMAKE_BINARY_DIR}/stage/bin

# System Setup

The following is my basic setup using Chirpstack.

## Gateway setup

I used Chirpstack v4.6.0 on my Waveshare gateway.  The Waveshare hat was connected to a Pi 4B.  I installed the stock openWRT images on an sdcard.  Chirp stack is well documented, start at the below link:

https://www.chirpstack.io/docs/chirpstack-gateway-os/install/raspberry-pi.html

Create a device profile.

Its important here to make sure your MAC Version and Regional parameters match what LoRa Basics Modem supports.  LoRaWAN v1.0.4 and RP002 v1.0.3.  Refer to Semtech's documentation on this but currently thats it.

![image](docs/images/chirp1.png)

Create an appliction and add a device.

As long as your on a small isolated network you can use whatever for Device EUI just make sure it matches what you have in the otaa_example.

![image](docs/images/chirp2.png)

Same with Application Key

![image](docs/images/chirp3.png)

Once everything is setup you should start to see frame and events for your application.  In this case a successful Join Request/Accept then uplinks.
![image](docs/images/chirp4.png)


**note:  Join times can be a bit slow, I believe with my configuration the concentrator was only scanning the first bank of channels 0-7 and the Lora Basics Modem library will go through the entire channel list 0-64.  I did not see any way to mask channel list in Lora Basics Modem API.    


## TODO

- Add GPS support for GNSS HAT
- New Modem test application using the smtc_modem_test stuff
