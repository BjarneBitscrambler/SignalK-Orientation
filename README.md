# SignalK-Orientation

## Overview
This library provides vessel and vehicle orientation information in [Signal K](https://signalk.org/) message format. Available orientation data include:
* Attitude (yaw, pitch, roll)
* Compass Heading and Magnetic Heading (corrected for deviation)
* Acceleration in 3 axes
* Turn Rate
* Pitch Rate
* Roll Rate

It uses a 9-axis combination accelerometer/magnetometer/gyroscope attached to an Espressif ESP32 or ESP8266 processor. Sensor fusion is performed by the ESP using a port of NXP's version 7 sensor fusion library, and formatted into Signal K by the SensESP library. SensESP also takes care of transferring the orientation data via WiFi to a Signal K server.

Using the hardware below, Signal K messages containing heading data can be output at up to 40 Hz.

## Hardware
Orientation sensing uses an NXP FXOS8700 and FXAS21002C/FXAS21002CQ combination sensor, like [the Adafruit 3463 module](https://www.adafruit.com/product/3463)

Processing and WiFi connection is provided by an ESP32 or ESP8266 module. It has been tested on a [d1_mini](https://www.wemos.cc/en/latest/d1/d1_mini.html), an [ESP32-WROVER-KIT](https://www.digikey.ca/en/products/detail/espressif-systems/ESP-WROVER-KIT-VB/8544301), and an [ESP-32-WROOM dev board](https://www.amazon.se/dp/B08CCYWZN3)

The software is adaptable to work with other orientation sensors, but the most straightfortward approach is to use the above already-tested hardware.

## Software
The PlatformIO development environment and Arduino framework are used, with standard Arduino libraries plus these two:
* [SensESP](https://github.com/SignalK/SensESP)
* [OrientationSensorFusion-ESP](https://github.com/BjarneBitscrambler/OrientationSensorFusion-ESP)

## Setup
Follow the instructions in the [SensESP README](https://github.com/SignalK/SensESP) to install Signal K and SensESP. The base installation of SensESP provides several built-in sensors - reporting information like *Uptime*, *Freemem*, and *IP Address* - which will be readable on your Signal K server once you have your hardware and software set up properly. For the most stress-free experience, it's recommended you don't try to use this library until after you have successfully built and seen the desired output in the Signal K Server.

After you have the basic setup working:
1. Start a new Project in PlatformIO for the Arduino and your processor platform
2. Copy the `platformio.ini` file from your working basic project (above) into your new project folder
3. Make one modification to your shiny new `platformio.ini`: add these two libraries to the *lib_deps* section, as follows. See this project's sample `platformio.ini` for more details and options.
```
lib_deps =
   [...]
   SignalK/SensESP
   https://github.com/BjarneBitscrambler/SignalK-Orientation.git
```
4. Replace the contents of your Project's `main.cpp` file with the contents of the example file that's included with this library (found in `examples/example_main.cpp`  Then edit your `main.cpp` to reflect the details of your particular setup (e.g. WiFi credentials, I2C pins connected to the sensor, etc). Be sure to read the comments in `main.cpp` that explain how to enable the various orientation parameters, setup the Signal K paths, and so forth.
5. Build, upload, and test your Project.

## Troubleshooting and Going Further
If you start with a basic configuration and proceed in small testable steps, it should go reasonably smoothly. If you run into difficulty, or just want to learn more, here are some resources:
* **Signal K** https://signalk.org/ for details on Signal K message contents, connectivity, units, display options.
* **Signal K Server** https://github.com/SignalK/signalk-server a Signal K server intended for Raspberry Pi, Beaglebone, or UDOO platforms.
* **SensESP** https://github.com/SignalK/SensESP for setting up an ESP32 or ESP8266 sensor and connecting it to a Signal K server. See also the Wiki at this location for additional advice on getting the built-in sensors reporting to the Signal K server.
* **Sensor Fusion** https://github.com/BjarneBitscrambler/OrientationSensorFusion-ESP has details on the sensor fusion algorithm and orientation sensor performance in the *Readme*, *Wiki*, and *Documentation* sections
* **this library** Additional information, particularly relating to performing magnetic calibration, is in this project's *Wiki*.
* **Contact Me** I can be contacted through the Discussions tab on the OrientationSensorFusion-ESP library: https://github.com/BjarneBitscrambler/OrientationSensorFusion-ESP/discussions

### ESP8266 Note Regarding Memory Use
The ESP8266 module has less RAM than the ESP32, and if you add multiple hardware sensors to a single module then you may run into a shortage of run-time memory. Symptoms of this problem include inability to access the ESP module's web interface. If the module has less than about 9000 bytes of freemem (as reported by one of the SensESP standard sensors, and seen in the Signal K Instrument Panel), then this may be causing difficulties. 

One fix is to compile the ESP software with `#define DEBUG_DISABLED`.  This causes the RemoteDebug library to ignore all the `debugI(), debugE()` etc calls throughout the SensESP code and saves about 8900 bytes of RAM. The downside to doing this is that any information one was gleaning from the serial terminal won't be sent anymore, however the Signal K and the sensor's web interface are unaffected.  To define DEBUG_DISABLED you can either uncomment the appropriate line in `RemoteDebug.h` and `RemoteDebug.cpp`, or the easier way is to add `-D DEBUG_DISABLED` to your list of `build_flags = ...` in   `platformio.ini`  Remember to do a clean before rebuilding.  

More memory-saving details can be found at https://github.com/SignalK/SensESP/issues/239

As of January 2021, the **d1_mini** runs when configured with one orientation sensor plus Signal K reports for heading, attitude, IP address, freemem, uptime, system Hz, magnetic calibration parameters, as well as monitoring one physical switch.  This is close to the maximum though, without taking extra steps such as disabing the debug messages as described above.
