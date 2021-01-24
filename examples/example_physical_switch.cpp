/** @file example_physical_switch.cpp
 *  @brief Demonstrating how to tie physical switch to a software action.
 * This file provides an example of how to use a physical switch to trigger
 * an action when using the Orientation library together with SensESP.
 */

#include <Arduino.h>
#include <Wire.h>

#include <sstream>
#include <string>

#include "sensesp_app.h"
#include "orientation_sensor.h"
#include "signalk_output.h"
/**
 * If using a button-press to cause an action, then include
 * digital_input.h, lambda_consumer.h, and debounce.h
 */
#include "sensors/digital_input.h"
#include "system/lambda_consumer.h"
#include "transforms/debounce.h"

// Sensor hardware details: I2C addresses and pins       
#define BOARD_ACCEL_MAG_I2C_ADDR    (0x1F) ///< I2C address on Adafruit breakout board
#define BOARD_GYRO_I2C_ADDR         (0x21) ///< I2C address on Adafruit breakout board
#if defined( ESP8266 )
  #define PIN_I2C_SDA (12)          // Adjust to your board. A value of -1
  #define PIN_I2C_SCL (14)          // will use default Arduino pins.
  #define PIN_SWITCH_CAL_SAVE (0)   // When at SWITCH_ACTIVE_STATE, saves mag calibration
  #define SWITCH_ACTIVE_STATE (0)   // Input is LOW when Switch is pushed
#elif defined( ESP32 )
  #define PIN_I2C_SDA (23)          // Adjust to your board. A value of -1
  #define PIN_I2C_SCL (25)          // will use default Arduino pins.
  #define PIN_SWITCH_CAL_SAVE (36)  // When brought LOW, will save magnetic calibration
  #define SWITCH_ACTIVE_STATE (0)   // Input is LOW when Switch is pushed
#endif

// SensESP builds upon the ReactESP framework. Every ReactESP application
// defines an "app" object vs defining a "main()" method.
ReactESP app([]() {

// Setup Serial port, and enable debug prints if in debug mode
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  /**
   * Create the global SensESPApp() object.
   * By passing the WiFi setup details in the constructor, rather than
   * relying on entering it in the device's web interface, we save about
   * 2496 bytes of heap memory (RAM). Another alternative is to use the
   * Builder pattern (sensesp_app_builder.h), but that saves only 1880 bytes.
   */
  sensesp_app = new SensESPApp(
      "SensESP_D1",         //hostname (name of this ESP device as advertised to SignalK)
      "mySSID",             //WiFi network SSID
      "myPassword",         //WiFi network password
      "192.168.1.4",        //IP address of network's Signal K server
      3000);                //port on which to connect to Signal K server

  /**
   * Create and initialize the Orientation data source.
   */
  auto* orientation_sensor = new OrientationSensor(
      PIN_I2C_SDA, PIN_I2C_SCL, BOARD_ACCEL_MAG_I2C_ADDR, BOARD_GYRO_I2C_ADDR);

  /*
   * Create the desired outputs from orientation sensor.
   */
  auto* sensor_heading = new OrientationValues(
      orientation_sensor, OrientationValues::kCompassHeading,
      100, "");
  sensor_heading->connect_to(
      new SKOutputNumber("navigation.headingCompass", ""));

  /**
   * Start of switch example. Copy this code section into your own `main.cpp,
   * Add three SensESP *.h files (see top of this example) for the digital
   * input, debounce, and consumer action functionality. Adjust the pin number
   * to whichever one you have a switch hooked up to, and recompile.
   *
   * The digital input pin is read periodically and if its state changes,
   * the next consumer (debounce transform) is notified. The switch read
   * interval and debounce period should be adjusted depending
   * on whether you need a quick response or immunity to accidental
   * pushes on the switch.
   *
   * The debounce transform sends its output to a *lambda consumer* function,
   * which in this example calls the orientation library's InjectCommand()
   * method. That causes the current magnetic calibration to be saved to
   * EEPROM.
   *
   * This example **physical button grounds the GPIO** pin when pushed,
   * so the GPIO has its pull-up resistor enabled.  One can instead use a
   * pull-down on the GPIO and tie the switch to logic high instead - just
   * adjust the code that sets up the digital input.
   *
   * For good noise immunity and CPU protection in a device to be used in
   * the field, I recommend putting a few passive components on the switch
   * circuit.  This is particularly a good idea if the wire length between
   * the CPU and the switch is longer than, say, 10 cm or there's a chance
   * of zapping the switch with static electricity.  There's quite a few
   * configurations commonly used for input protection. One is a resistor
   * in series between the switch and the GPIO pin of about 1000 ohms
   * (the pull-up/pull-down resistance for ESP32 pins is 45 k, so 1 k will
   * overcome that nicely), and then an ESD protection diode rated between
   * 3V3 and 5V0 between the GPIO pin and board ground. An alternative is
   * to use a small-value capacitor instead of the ESD diode - just check
   * that the RC time constant together with the 1k resistor will be OK.
   */
  // Monitor a button every read_interval ms for CHANGEs in state.
  // Note that no web interface path is supplied, so interval is not adjustable.
  // Note INPUT_PULLUP may need to change depending on how button is wired.
  const int kReadInterval = 100;
  auto* button_watcher = new DigitalInputChange(
      PIN_SWITCH_CAL_SAVE, INPUT_PULLUP, CHANGE, kReadInterval, "");
  // Create a debounce transform, also with no web interface.
  const int kDebounceDelay =
      350;  // only react to pushes >350 ms + kReadInterval
  auto* debounce = new DebounceInt(kDebounceDelay, "");
  // Define the action taken when button is active and debounce has elapsed.
  // Provide it with the context of orientation_sensor so it can access save
  // fcn.
  auto save_mcal_function = [orientation_sensor](int input) {
    if (input == SWITCH_ACTIVE_STATE) {
      orientation_sensor->sensor_interface_->InjectCommand("SVMC");
      debugI("Mag Cal saved");
    }
  };
  auto* button_consumer = new LambdaConsumer<int>(save_mcal_function);
  // Connect the button -> debounce -> save magnetic calibration fcn
  button_watcher->connect_to(debounce)->connect_to(button_consumer);
  /**
   * End of example physical switch section.
   */

  // Start the SensESP application running
  sensesp_app->enable();
});
