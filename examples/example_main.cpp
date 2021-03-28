/** @file main.cpp
 *  @brief Orientation output in Signal K format via SensESP.
 * This file provides examples for using the Orientation library together
 * with SensESP to report vessel orientation data to a Signal K server.
 * 
 * Intended hardware is an ESP32 or ESP8266 platform and an FXOS8700/FXAS21002
 * combination accelerometer/magnetometer/gyroscope.
 * 
 * The examples include:
 *   * Compass Heading output
 *   * Attitude (yaw, pitch, roll) output
 *   * Magnetic Heading (Compass reading corrected for deviations)
 *   * Physical switch to trigger saving of magnetic calibration
 *   * Temperature output (taken from the sensor IC and corrected)
 *   * Acceleration in X,Y, Z axes
 *   * Turn, Pitch, and Roll Rates
 * Some example outputs are commented out by default: read the associated
 * comments for details.
 */

#include <Arduino.h>
#include <Wire.h>

#include <sstream>
#include <string>

#include "sensesp_app.h"
#include "orientation_sensor.h"
#include "signalk_output.h"
/**
 * If using a button-press to save Magnetic Calibration, then include
 * digital_input.h, lambda_consumer.h, and debounce.h
 */
#include "sensors/digital_input.h"
#include "system/lambda_consumer.h"
#include "transforms/debounce.h"
/**
 * If wanting to correct compass readings for mounting offsets or
 * residual deviations after magnetic calibration, then need either
 * angle correction transform (which provides a straight offset)
 * or a curve interpolator transform (which provides interpolation
 * to a user-supplied set of points).
 */
#include "transforms/angle_correction.h"
#include "transforms/curveinterpolator.h"
/**
 * If using the Temperature report, then include the linear transform
 * for calibrating the temperature readings.
 */
//#include "transforms/linear.h"

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

// How often orientation parameters are published via Signal K message
// If a report interval is saved for a particular sensor path (via the web
// interface), that overrides the following #define for that report.
#define ORIENTATION_REPORTING_INTERVAL_MS (100)

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
      "SensESP_D1",         //name of this ESP device as sent to SignalK
      "mySSID",             //WiFi network SSID
      "myPassword",         //WiFi network password
      "192.168.1.4",        //IP address of network's Signal K server
      3000);                //port on which to connect to Signal K server

  /**
   * The "SignalK path" identifies this sensor to the Signal K server. Leaving
   * this blank would indicate this particular sensor or transform does not
   * broadcast Signal K data.
   * If you have multiple sensors connected to your microcontroller (ESP),
   * each of them will probably have its own Signal K path variable. For
   * example, if you have two propulsion engines, and you want the RPM of
   * each of them to go to Signal K, you might have
   * sk_path_portEngine = "propulsion.port.revolutions" and
   * sk_path_starboardEngine = "propulsion.starboard.revolutions"
   * To find valid Signal K Paths look at this link (or later version):
   * @see https://signalk.org/specification/1.5.0/doc/vesselsBranch.html
   *
   * Vessel heading can be reported as headingCompass (uncorrected for
   * deviation), headingMagnetic (corrected for deviations),
   * or as part of an attitude data group (i.e. yaw, pitch, roll).
   * All three paths are defined in the Signal K spec and have default
   * display widgets in the Signal K Instrument Panel.
   */
  const char* kSKPathHeadingCompass  = "navigation.headingCompass";
  const char* kSKPathHeadingMagnetic = "navigation.headingMagnetic";
  const char* kSKPathAttitude        = "navigation.attitude";
  /**
   * This example reports heading, pitch, and roll. If you want other parameters
   * as well, uncomment the appropriate SKpath(s) from the following.
   * Signal K v1.5 does not describe paths for roll rate and pitch rate
   * so these are provided using the same pattern as for rateOfTurn.
   * Signal K v1.5 says path for temperature can include zone.
   * Replace ecompass with a different zone if desired.
   * Signal K v1.5 does not describe a path for acceleration.
   */
  //const char* kSKPathTurnRate    = "navigation.rateOfTurn";
  //const char* kSKPathRollRate    = "navigation.rateOfRoll";
  //const char* kSKPathPitchRate   = "navigation.rateOfPitch";
  //const char* kSKPathTemperature =
  //               "environment.inside.ecompass.temperature";
  //const char* kSKPathAccel       = "sensors.accelerometer.accel_xyz";
  /**
   * The following SKpaths are useful when performing magnetic calibration,
   * and for confirming that the current magnetic environment of the sensor
   * is unchanged from the most recent saved calibration. None of these
   * parameters has a defined path in Signal K, so they may be changed to suit.
   * 
   * For more details and suggestions on how to perform magnetic calibration,
   * see the Wiki at
   * @see https://github.com/BjarneBitscrambler/SignalK-Orientation/wiki
   */
  const char* kSKPathMagFit          = "orientation.calibration.magfit";
  const char* kSKPathMagFitTrial     = "orientation.calibration.magfittrial";
  const char* kSKPathMagSolver       = "orientation.calibration.magsolver";
  //const char* kSKPathMagInclination  = "orientation.calibration.maginclination";
  //const char* kSKPathMagBValue      = "orientation.calibration.magmagnitude";
  //const char* kSKPathMagBValueTrial = "orientation.calibration.magmagnitudetrial";
  //const char* kSKPathMagNoise       = "orientation.calibration.magnoise";
  //const char* kSKPathMagCalValues   = "orientation.calibration.magvalues";

  /**
   * If you are creating a new Signal K path that does not
   * already exist in the specification, it is best to
   * define "metadata" that describes your new value. This
   * metadata will be reported to the Signal K server the first
   * time your sensor reports its value(s) to the server.
   */
  // Uncomment from the following example metadata as needed, or create
  // your own as needed.
  //   SKMetadata* metadata_accel = new SKMetadata();
  //   metadata_accel->description_ = "Acceleration in X,Y,Z axes";
  //   metadata_accel->display_name_ = "Accelerometer";
  //   metadata_accel->short_name_ = "Accel";
  //   metadata_accel->units_ = "m/s^2";
  //
  //   SKMetadata* metadata_rate_of_roll = new SKMetadata();
  //   metadata_rate_of_roll->description_ =
  //        "Rate of Roll about bow-stern axis";
  //   metadata_rate_of_roll->display_name_ = "Roll Rate";
  //   metadata_rate_of_roll->short_name_ = "Roll Rate";
  //   metadata_rate_of_roll->units_ = "rad/s";
  //
  //   SKMetadata* metadata_rate_of_pitch = new SKMetadata();
  //   metadata_rate_of_pitch->description_ =
  //        "Rate of Pitch about port-starboard axis";
  //   metadata_rate_of_pitch->display_name_ = "Pitch Rate";
  //   metadata_rate_of_pitch->short_name_ = "Pitch Rate";
  //   metadata_rate_of_pitch->units_ = "rad/s";
  //
  //   SKMetadata* metadata_temperature = new SKMetadata();
  //   metadata_temperature->description_ =
  //        "Temperature reported by orientation sensor";
  //   metadata_temperature->display_name_ = "Temperature at eCompass";
  //   metadata_temperature->short_name_ = "Temp";
  //   metadata_temperature->units_ = "K";

  /**
   * The "Configuration path" is combined with "/config" to formulate a URL
   * used by the RESTful API for retrieving or setting configuration data.
   * It is ALSO used to specify a path to the file system
   * where configuration data is saved on the MCU board. It should
   * ALWAYS start with a forward slash if specified. 
   * The max length for a config path is 32 characters, due to a limitation
   * in the SPIFFS filesystem.
   * If the config path is left blank,
   * that indicates this sensor or transform does not have any
   * configuration to save, or that you're not interested in doing
   * run-time configuration.
   * These two are necessary until a method is created to synthesize them.
   * 
   * Note the hardware sensor itself has no run-time configurable items.
   * Note the empty "" for configuring the SK paths for attitude and 
   * heading: this is because the paths for these parameters are
   * prescribed by the SK spec, and default instruments expect these
   * paths. You can override them, but will then need to define your
   * own instruments to display the data.
   * 
   * Below arrangement of config paths yields this web interface structure:
   * 
       sensors->attitude
                       ->settings (adjusts report interval, saves mag cal)
              ->heading
                       ->settings (adjusts report interval, saves mag cal)
                       ->deviation      (adjusts compass deviation)
   * 
   */
  const char* kConfigPathAttitude_SK = "";
  const char* kConfigPathAttitude    = "/sensors/attitude/settings";
  const char* kConfigPathHeading_SKC = "";
  const char* kConfigPathHeading_SKM = "";
  const char* kConfigPathHeading     = "/sensors/heading/settings";
  const char* kConfigPathHeadingDev  = "/sensors/heading/deviation";
  // This example shows attitude and compass heading. If you want other parameters
  // as well, uncomment and modify the appropriate path(s) from the following 
  // or create new paths as needed.
  //   const char* kConfigPathTurnRate_SK    = "/sensors/rateOfTurn/sk";
  //   const char* kConfigPathTurnRate       = "/sensors/rateOfTurn/settings";
  //   const char* kConfigPathAccelXYZ       = "/sensors/acceleration/settings";
  //   const char* kConfigPathAccelXYZ_SK    = "/sensors/acceleration/sk";
  //   const char* kConfigPathTemperature    = "/sensors/temperature/settings";
  //   const char* kConfigPathTemperature_SK = "/sensors/temperature/sk";

  /**
   * Create and initialize the Orientation data source.
   * This uses a 9 Degrees-of-freedom combination sensor that provides multiple
   * orientation parameters. Selection of which particular parameters are
   * output is performed later when the value producers are created.
   * 
   * Magnetic Calibration occurs during regular runtime. After power-on, move
   * the sensor through a series of rolls, pitches and yaws. After enough
   * readings have been collected (takes 15-30 seconds when rotating the sensor
   * by hand) then the sensor should be calibrated.
   * A Magnetic Calibration can be saved in non-volatile memory so it will be
   * loaded at the next power-up. To save a calibration, use the
   * value_settings->Save_Mag_Cal entry in the sensor web interface.
   * A calibration will be valid until the sensor's magnetic environment
   * changes.
   */
  auto* orientation_sensor = new OrientationSensor(
      PIN_I2C_SDA, PIN_I2C_SCL, BOARD_ACCEL_MAG_I2C_ADDR, BOARD_GYRO_I2C_ADDR);

  /*
   * Create the desired outputs from the orientation sensor. Note that the physical
   * sensor is read at whatever rate is specified in the Sensor Fusion library's
   * build.h file (#define FUSION_HZ), currently set to 40 Hz. Fusion
   * calculations are run at that same rate. This is different than, and
   * usually faster than, the rate at which orientation parameters are output.
   * Reporting orientation values within SensESP can happen at any desired
   * rate, though if it is more often than the fusion rate then
   * there will be duplicated values. This example uses a 10 Hz output rate.
   * It is not necessary that all the values be output at the same rate (for
   * example, it likely makes sense to report temperature at a slower rate).
   */
  
  // Create the Compass Heading and Magnetic Heading outputs. The difference
  // between the two, in this example, is that the Magnetic Heading passes
  // through a transform allowing one to correct for a fixed offset (such
  // as occurs when the sensor's axis is not perfectly parallel with the 
  // vessel's stern-bow axis).
  auto* sensor_heading = new OrientationValues(
      orientation_sensor, OrientationValues::kCompassHeading,
      ORIENTATION_REPORTING_INTERVAL_MS, kConfigPathHeading);
  sensor_heading->connect_to(
      new SKOutputNumber(kSKPathHeadingCompass, kConfigPathHeading_SKC))
      //pass through transform. Set initial offset to 0.0 radians.
      ->connect_to( new AngleCorrection( 0.0, 0.0, kConfigPathHeadingDev) )
      //an optional, more complex transform is a CurveInterpolator
      //->connect_to( new CurveInterpolator( NULL, "/sensors/heading/Magnetic") )
      ->connect_to(
      new SKOutputNumber(kSKPathHeadingMagnetic, kConfigPathHeading_SKM));
  
  // Create the Attitude output (yaw, pitch, roll). Note that this
  // output does not pass through any transform to correct for residual
  // deviation due to e.g. mounting offsets.
  auto* sensor_attitude = new AttitudeValues(
      orientation_sensor, ORIENTATION_REPORTING_INTERVAL_MS,
      kConfigPathAttitude);
  sensor_attitude->connect_to(
      new SKOutputAttitude(kSKPathAttitude, kConfigPathAttitude_SK));

  /**
   * The following outputs are useful when calibrating. See the wiki at
   * @see https://github.com/BjarneBitscrambler/SignalK-Orientation/wiki
   * for details on how to interpret the values. None are recognized
   * in the Signal K spec, so there is no prescribed SK path that they
   * need to be sent to. 
   * 
   * Because there are quite a few parameters, and they are likely only
   * referred to infrequently (i.e. when calibrating, or when magnetic
   * disturbances are suspected), you may want to configure the Signal K
   * instrument panel to display these paths on a secondary screen, 
   * separate from the primary navigation screen.
   */
  auto* sensor_cal_fit = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalFitInUse,
      ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  sensor_cal_fit->connect_to(
      new SKOutputNumber(kSKPathMagFit, ""));

  auto* sensor_cal_candidate = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalFitTrial,
      ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  sensor_cal_candidate->connect_to(
      new SKOutputNumber(kSKPathMagFitTrial, ""));

  auto* sensor_cal_order = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalAlgorithmSolver,
      ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  sensor_cal_order->connect_to(
      new SKOutputNumber(kSKPathMagSolver, ""));

  // auto* sensor_mag_inclination = new OrientationValues(
  //     orientation_sensor, OrientationValues::kMagInclination,
  //     ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_mag_inclination->connect_to(
  //     new SKOutputNumber(kSKPathMagInclination, ""));

  // auto* sensor_mag_b_value = new OrientationValues(
  //     orientation_sensor, OrientationValues::kMagFieldMagnitude,
  //     ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_mag_b_value->connect_to(
  //     new SKOutputNumber(kSKPathMagBValue, ""));

  // auto* sensor_mag_b_value_trial = new OrientationValues(
  //     orientation_sensor, OrientationValues::kMagFieldMagnitudeTrial,
  //     ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_mag_b_value_trial->connect_to(
  //     new SKOutputNumber(kSKPathMagBValueTrial, ""));

  // auto* sensor_mag_noise = new OrientationValues(
  //     orientation_sensor, OrientationValues::kMagNoiseCovariance,
  //     ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_mag_noise->connect_to(
  //     new SKOutputNumber(kSKPathMagNoise, ""));

  // This report is a consolidation of all the above magnetic cal
  // values and will need a custom instrument to display.
  // auto* sensor_mag_cal = new MagCalValues(
  //     orientation_sensor, ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_mag_cal->connect_to(
  //     new SKOutputMagCal(kSKPathMagCalValues, ""));

  /**
   * Following section monitors a physical switch that, when pressed,
   * saves the sensor's current magnetic calibration to non-volatile
   * memory. Comment/uncomment the following as needed.
   */
  // Monitor a button every read_interval ms for CHANGEs in state.
  // No web interface path is supplied, so interval won't be adjustable.
  // Note INPUT_PULLUP may need to change depending on how button is wired.
  const int kReadInterval = 100;
  auto* button_watcher = new DigitalInputChange(
      PIN_SWITCH_CAL_SAVE, INPUT_PULLUP, CHANGE, kReadInterval, "");
  // Create a debounce transform, also with no web interface.
  const int kDebounceDelay = 350; // only react to pushes >350 ms + kReadInterval
  auto* debounce = new DebounceInt(kDebounceDelay, "");
  // Define the action taken when button is active and debounce has elapsed.
  // Provide it with the context of orientation_sensor so it can access save fcn.
  auto save_mcal_function = [orientation_sensor](int input) {
    if (input == SWITCH_ACTIVE_STATE) {
      orientation_sensor->sensor_interface_->SaveMagneticCalibration();
      debugI("Mag Cal saved");
    }
  };
  auto* button_consumer = new LambdaConsumer<int>(save_mcal_function);
  // Connect the button -> debounce -> save magnetic calibration fcn
  button_watcher->connect_to(debounce)->connect_to(button_consumer);
  /**
   * End of physical switch section.
   */

  // This example reports attitude and heading. If you want other parameters
  // as well, uncomment the appropriate connections from the following.
  //   auto* sensor_turn_rate = new OrientationValues(
  //       orientation_sensor, OrientationValues::kRateOfTurn,
  //       ORIENTATION_REPORTING_INTERVAL_MS, kConfigPathTurnRate);
  //   sensor_turn_rate->connect_to(
  //       new SKOutputNumber(kSKPathTurnRate, kConfigPathTurnRate_SK));

  //   auto* sensor_roll_rate = new OrientationValues(
  //       orientation_sensor, OrientationValues::kRateOfRoll,
  //       ORIENTATION_REPORTING_INTERVAL_MS, kConfigPathRollRate);
  //   sensor_roll_rate->connect_to(
  //       new SKOutputNumber(kSKPathRollRate, kConfigPathRollRate_SK));

  //   auto* sensor_pitch_rate = new OrientationValues(
  //       orientation_sensor, OrientationValues::kRateOfPitch,
  //       ORIENTATION_REPORTING_INTERVAL_MS, kConfigPathPitchRate);
  //   sensor_pitch_rate->connect_to(
  //       new SKOutputNumber(kSKPathPitchRate, kConfigPathPitchRate_SK));

  // TODO - it makes sense to send all three accel values (XYZ) in
  // one SK package. The needed data structure is not yet defined in
  // SensESP. The following sends the X accel as a single value.
  //   auto* sensor_accel_x = new OrientationValues(
  //       orientation_sensor, OrientationValues::kAccelerationX,
  //       ORIENTATION_REPORTING_INTERVAL_MS, kConfigPathAccelXYZ);
  //   sensor_accel_x->connect_to(
  //       new SKOutputNumber(kSKPathAccel, kConfigPathAccelXYZ_SK));

  //   auto* sensor_temperature =
  //       new OrientationValues(orientation_sensor,
  //       OrientationValues::kTemperature,
  //                             1000, kConfigPathTemperature);
  //   sensor_temperature
  //       ->connect_to(new Linear(1.0, 0.0, "/sensors/temperature/calibrate"))
  //  Temperature readings are passed through a linear transform
  //  to allow for calibration/linearization via web interface. Other
  //  transforms are available. Ensure you #include the appropriate file(s).
  //       ->connect_to(new SKOutputNumber(
  //           kSKPathTemperature, kConfigPathTemperature_SK,
  //           metadata_temperature));

  /**
   *  Relationship of the Axes and the terminology:
   * If the sensor is mounted with the X-axis pointing to the bow of the boat
   * and the Y-axis pointing to Port, then Z points up and the normal marine
   * conventions apply. The wiki has details:
   * @see https://github.com/BjarneBitscrambler/SignalK-Orientation/wiki
   * 
   * If the sensor is mounted differently, or you prefer an alternate nomenclature,
   * the Get___() methods in sensor_fusion_class.cpp can be adjusted.
  */

  // Start the SensESP application running
  sensesp_app->enable();
});
