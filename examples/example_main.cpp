/** @file main.cpp
 *  @brief Orientation output in Signal K format via SensESP.
 * This file provides examples for using the Orientation library together
 * with SensESP to report vessel orientation data to a Signal K server.
 * 
 * Intended hardware is an ESP32 platform and an FXOS8700/FXAS21002
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


#include <memory>

/**
 * Mandatory SensESP and Orientation library headers
 */
#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

#include "orientation_sensor.h"
#include "signalk_orientation.h"

/**
 * If using a button-press to save Magnetic Calibration, then include
 * digital_input.h, lambda_consumer.h, and debounce.h   TODO use the newer SensESP Button service
 */
#include "sensesp/transforms/debounce.h"
/**
 * If wanting to correct compass readings for mounting offsets or
 * residual deviations after magnetic calibration, then need either
 * angle correction transform (which provides a straight offset)
 * or a curve interpolator transform (which provides interpolation
 * to a user-supplied set of points).
 */
#include "sensesp/transforms/angle_correction.h"
#include "sensesp/transforms/curveinterpolator.h"
/**
 * If using the Temperature report, then include the linear transform
 * for calibrating the temperature readings.
 */
#include "sensesp/transforms/linear.h"

// Sensor hardware details: I2C addresses and pins       
#define BOARD_ACCEL_MAG_I2C_ADDR    (0x1F) ///< I2C address on Adafruit breakout board
#define BOARD_GYRO_I2C_ADDR         (0x21) ///< I2C address on Adafruit breakout board
#define PIN_I2C_SDA (23)          // Adjust to your board. A value of -1
#define PIN_I2C_SCL (25)          //   will use default Arduino pins.
#define PIN_SWITCH_CAL_SAVE (36)  // Optional switch attached to this pin saves magnetic calibration
#define SWITCH_ACTIVE_STATE (0)   // Input is LOW when Switch is pushed

// How often orientation parameters are published via Signal K message
// If a report interval is saved for a particular sensor path (via the web
// interface), that overrides the following #define for that report.
#define ORIENTATION_REPORTING_INTERVAL_MS (100)

using namespace sensesp;

class DeviationInterpolator : public CurveInterpolator {
 public:
  DeviationInterpolator(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the compass values to magnetic heading
    //The actual values should be edited to suit your own actual Deviation table, 
    // either here or using the SensESP web configuration page
    clear_samples();
    // add_sample(CurveInterpolator::Sample(compass reading, magnetic heading));
    //  values are in radians.
    add_sample(CurveInterpolator::Sample(0.0, 0.0));
    add_sample(CurveInterpolator::Sample(6.3, 6.3));  //default Deviation table is straight 1:1 conversion
  }
};



// The setup function performs one-time application initialization.
void setup() {
  //without following, get runtime message
  // I (2147) ARDUINO: LEDC attached to pin 2 (channel 0, resolution 8)
  // E (2149) ARDUINO: IO 0 is not set as GPIO. Execute digitalMode(0, OUTPUT) first.
  // E (2149) ARDUINO: IO 2 is not set as GPIO. Execute digitalMode(2, OUTPUT) first.
  // E (2149) ARDUINO: IO 4 is not set as GPIO. Execute digitalMode(4, OUTPUT) first.
  pinMode(0, OUTPUT); 
  pinMode(2, OUTPUT); //this doesn't prevent the GPIO2 complaint.
  pinMode(4, OUTPUT);

  SetupLogging(ESP_LOG_INFO);

 /**
   * Create the global SensESPApp() object.
   * By passing the WiFi setup details in the constructor, rather than
   * relying on entering it in the device's web interface, we save about
   * 2496 bytes of heap memory (RAM). Another alternative, used below, is the
   * Builder pattern (sensesp_app_builder.h), which saves about 1880 bytes.
   */

  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("eCompass")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("10.0.0.27", 3000)
                    //SensESP has several builtin sensors, e.g. freemem, uptime, IP address
                    //  Optionally enable them here to output their values in SK reports.
                    ->enable_uptime_sensor()
                    ->enable_ip_address_sensor()
                    ->enable_free_mem_sensor()
                    ->enable_system_hz_sensor()
//                    ->enable_wifi_signal_sensor()
                    ->get_app();

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
   * @see https://signalk.org/specification/1.7.0/doc/vesselsBranch.html
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
   * Signal K v1.7 does not describe paths for roll rate and pitch rate
   * so these are provided using the same pattern as for rateOfTurn.
   */
  const char* kSKPathTurnRate    = "navigation.rateOfTurn";
//  const char* kSKPathRollRate    = "navigation.rateOfRoll";
//  const char* kSKPathPitchRate   = "navigation.rateOfPitch";
  /**
   * Signal K v1.7 says path for temperature can include zone.
   * Replace ecompass with a different zone if desired.
   */
  const char* kSKPathTemperature =
                 "environment.inside.ecompass.temperature";
  /**
   * Signal K v1.7 does not describe a path for acceleration.
   */
  const char* kSKPathAccel       = "sensors.accelerometer.accel_xyz";
  
  /**
   * The following SKpaths are useful when performing magnetic calibration,
   * and for confirming that the current magnetic environment of the sensor
   * is unchanged from the most recent saved calibration. None of these
   * parameters have predefined paths in the Signal K spec, so may be changed to suit.
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
  const char* kSKPathMagNoise       = "orientation.calibration.magnoise";
  //const char* kSKPathMagCalValues   = "orientation.calibration.magvalues";

  /**
   * If you are creating a new Signal K path that does not
   * already exist in the specification, it is best to
   * define "metadata" that describes your new value. This
   * metadata will be reported to the Signal K server the first
   * time your sensor reports its value(s) to the server.
   *
   * Uncomment from the following example metadata as needed, or create
   * your own.
   */
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
     SKMetadata* metadata_temperature = new SKMetadata();
     metadata_temperature->description_ =
          "Temperature reported by orientation sensor";
     metadata_temperature->display_name_ = "Temperature at eCompass";
     metadata_temperature->short_name_ = "Temp";
     metadata_temperature->units_ = "K";

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
                       ->deviation (adjusts compass deviation with Curve Interpolator)
                       ->offset    (adjusts compass deviation with single value)
                       ->settings  (adjusts report interval, saves mag cal)
   * 
   */
  const char* kConfigPathAttitude_SK = "";
  const char* kConfigPathAttitude    = "/sensors/attitude/settings";
  const char* kConfigPathHeading_SKC = "";
  const char* kConfigPathHeading_SKM = "";
  const char* kConfigPathHeading     = "/sensors/heading/settings";
  const char* kConfigPathHeadingDev  = "/sensors/heading/deviation";
  const char* kConfigPathHeadingOffset = "/sensors/heading/offset";
  /* The above provides a web interface for attitude and compass heading. 
   * For other parameters, uncomment and modify the appropriate path(s) from the following 
   * or create new paths as needed.
   */
  const char* kConfigPathTurnRate_SK    = "";
  const char* kConfigPathTurnRate       = "";
  //   const char* kConfigPathAccelXYZ       = "/sensors/acceleration/settings";
  //   const char* kConfigPathAccelXYZ_SK    = "/sensors/acceleration/sk";
  //   const char *kConfigPathRollRate = "/sensors/rollRate/settings";
  //   const char *kConfigPathRollRate_SK = "/sensors/rollRate/sk";
  //   const char *kConfigPathPitchRate = "/sensors/pitchRate/settings";
  //   const char *kConfigPathPitchRate_SK = "/sensors/pitchRate/sk";
  const char* kConfigPathTemperature    = "/sensors/temperature/settings";
  const char* kConfigPathTemperatureCal = "/sensors/temperature/calibrate";
  const char* kConfigPathTemperature_SK = "/sensors/temperature/sk";


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
   * sensors/heading/settings->Save_Mag_Cal entry in the sensor web interface, AND/OR
   * enable and use the optional hardware switch mentioned later in this code.
   * A calibration will be valid until the sensor's magnetic environment
   * changes.
   */
  auto* orientation_sensor = new OrientationSensor(
      PIN_I2C_SDA, PIN_I2C_SCL, BOARD_ACCEL_MAG_I2C_ADDR, BOARD_GYRO_I2C_ADDR);
  const int fusionIntervalMs = 1000 / orientation_sensor->GetFusionRateHz();
  event_loop()->onRepeat( fusionIntervalMs,
                          [orientation_sensor]() { orientation_sensor->ReadAndProcessSensors(); }
                        );
  /*
   * Create the desired outputs from the orientation sensor. Note that the physical
   * sensor is read at whatever rate is specified in the Sensor Fusion library's
   * build.h file (#define FUSION_HZ), currently set to 40 Hz. Fusion
   * calculations are run at that same rate. This is different than, and
   * usually faster than, the rate at which orientation parameters are output.
   * Reporting orientation values within SensESP can happen at any desired
   * rate, though if it is more often than the fusion rate then
   * there will be duplicated values. This example uses a 10 Hz output rate, set
   * via #define ORIENTATION_REPORTING_INTERVAL_MS. The rate may be overridden
   * via a parameter's Value Settings->Report Interval entry in the web interface.
   * It is not necessary that all the values be output at the same rate (for
   * example, it likely makes sense to report temperature at a slower rate).
   */

    /* Enable Compass Heading and Magnetic Heading outputs. The difference
     * between the two, in this example, is that the Compass Heading is only
     * corrected for fixed mounting offsets (such as occurs when the sensor's 
     * axis is not perfectly parallel with the vessel's stern-bow axis) - 
     * Deviation corrections are not applied to the Compass Heading.
     * Magnetic Heading, on the other hand, has passed through one or more 
     * transforms to correct for other fixed or variable deviations. The
     * Curve Interpolator transform accepts pairs of (input,output) values
     * using the web interface, and uses them as a lookup table to provide 
     * a corresponding output value for a given input value. Linear interpolation
     * is performed when an exact input value is not found in the table. By using
     * multiple (input,output) value pairs, one can approximate an arbitrary
     * transform function. Note that with SensESP v1 only about 10 pairs of values
     * are accepted by the standard SensESP Curve Interpolator; if you need more, then
     * up to at least 37 pairs are suported by the source code at 
     * https://github.com/BjarneBitscrambler/SensESP.git#IncreaseCurveIntPoints
     * The relevant necessary changes are in http.cpp, configurable.cpp, and
     * curveinterpolator.cpp. Note that this modification is based on version
     * 1.0 of SensESP; initial testing with SensESP v2 indicates that the limitation
     * has been removed and the modified source code is no longer needed.
     */

// Create sensor for compass heading output     
  auto* sensor_heading = new OrientationValues(
      orientation_sensor, OrientationValues::kCompassHeading);

  auto compass_heading = std::make_shared<RepeatSensor<float>>(
      ORIENTATION_REPORTING_INTERVAL_MS,
      [sensor_heading]() { return sensor_heading->ReportValue(); });

/* No need to provide metadata, as Compass Heading is already defined in the Signal K spec.
  auto compass_metadata = std::make_shared<SKMetadata>();
  compass_metadata->units_ = "rad";  //don't specify "radians" as Signal K only recognizes "rad" 
  compass_metadata->description_ = "Uncorrected eCompass heading";
  compass_metadata->display_name_ = "Compass heading";
  compass_metadata->short_name_ = "C";
*/
  auto compass_sk_output = std::make_shared<SKOutput<float>>(
      "navigation.headingCompass",    // Signal K path
      ""                              // configuration path
      //compass_metadata 
      );
/* ConfigItem(compass_sk_output)
      ->set_title("Compass Heading")
      ->set_description("Uncorrected eCompass Reading")
      ->set_sort_order(200);
*/

//Set up an output for the Magnetic Heading, which will use the processed Compass Heading as an input
/* No need to provide metadata, as Magnetic Heading is already defined in the Signal K spec.
  auto magneticheading_metadata = std::make_shared<SKMetadata>();
  magneticheading_metadata->units_ = "rad";  //don't specify "radians" as Signal K only recognizes "rad" 
  magneticheading_metadata->description_ = "eCompass Heading corrected for Deviation";
  magneticheading_metadata->display_name_ = "Magnetic heading";
  magneticheading_metadata->short_name_ = "M";
*/
  auto magneticheading_sk_output = std::make_shared<SKOutput<float>>(
      "navigation.headingMagnetic",    // Signal K path
      ""  // configuration path
      //magneticheading_metadata
  );
/*  ConfigItem(magneticheading_sk_output)
      ->set_title("Magnetic Heading")
      ->set_description("Compass Reading corrected for Deviation")
      ->set_sort_order(201);
*/
 // Connect compass output to Signal K transforms and outputs
   compass_heading
    // Adjust for mounting offsets - Pi/2 rotation in this example
    ->connect_to(new AngleCorrection((PI/2.0), 0.0, kConfigPathHeadingOffset))
    // Output the Compass heading
    ->connect_to(compass_sk_output)
      /* Correct for Deviation. 
      * CurveInterpolator applies deviation corrections that can be specified
      * at points within the [0..2Pi] range, and interpolated for points 
      * between the specified ones. Lets you define a Deviation Table.
      * Use the SensESP server's Configuration interface to enter a Compass Deviation table,
      * or hard-code it in the DeviationInterpolator constructor (near top of this file)
      */
    ->connect_to( new DeviationInterpolator("/sensors/heading/deviation") )
    // Normalize to [0..2Pi] after interpolation
    ->connect_to(new AngleCorrection(0.0, 0.0, ""))
    // Output the Magnetic heading
    ->connect_to(magneticheading_sk_output);

 
    /* Enable Attitude output (yaw, pitch, roll). Note that this
   * output does not pass through any transform to correct for residual
   * deviation due to e.g. mounting offsets.
   */
/*  auto* sensor_attitude = new AttitudeValues(
      orientation_sensor, ORIENTATION_REPORTING_INTERVAL_MS,
      kConfigPathAttitude);
  sensor_attitude->connect_to(
      new SKOutputAttitude(kSKPathAttitude, kConfigPathAttitude_SK));
*/
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

  // Create sensor for Magnetic Calibration Fit output     
  auto* sensor_magcalfit = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalFitInUse);

  auto magcalfit = std::make_shared<RepeatSensor<float>>(
      4000, //update output value every 4000ms
      [sensor_magcalfit]() { return sensor_magcalfit->ReportValue(); });

  // Need to provide metadata, as Mag Cal related values are not defined in the Signal K spec.
  auto magcalfit_metadata = std::make_shared<SKMetadata>();
  magcalfit_metadata->units_ = "unitless";   
  magcalfit_metadata->description_ = "Measure of how well data fits current Magnetic Calibration";
  magcalfit_metadata->display_name_ = "Magnetic Calibration Fit";
  magcalfit_metadata->short_name_ = "MagCalFit";

  auto magcalfit_output = std::make_shared<SKOutput<float>>(
      kSKPathMagFit,   // Signal K path
      "",  // configuration path
      magcalfit_metadata
  );

  ConfigItem(magcalfit_output)
      ->set_title("Compass Heading")
      ->set_description("Uncorrected eCompass Reading")
      ->set_sort_order(200);

  magcalfit->connect_to(magcalfit_output);

//  auto* sensor_cal_candidate = new OrientationValues(
//      orientation_sensor, OrientationValues::kMagCalFitTrial,
//      ORIENTATION_REPORTING_INTERVAL_MS * 19);
//  auto cal_trial_metadata = std::make_shared<SKMetadata>("unitless", "Magnetic Calibration Fit Trial");
//  auto cal_trial_output = std::make_shared<SKOutput<float>>(
//      kSKPathMagFitTrial,   // Signal K path
//      "",  // configuration path
//      cal_trial_metadata
//  );
//  sensor_cal_candidate->connect_to(cal_trial_output);

//  auto* sensor_cal_order = new OrientationValues(
//      orientation_sensor, OrientationValues::kMagCalAlgorithmSolver,
 //     ORIENTATION_REPORTING_INTERVAL_MS * 99);
//  auto cal_solver_metadata = std::make_shared<SKMetadata>("unitless", "Magnetic Calibration Solver Algorithm");
//  auto cal_solver_output = std::make_shared<SKOutput<float>>(
 //     kSKPathMagSolver,   // Signal K path
//      "",  // configuration path
//      cal_solver_metadata
//  );
//  sensor_cal_order->connect_to(cal_solver_output);
//   auto* sensor_mag_inclination = new OrientationValues(
//       orientation_sensor, OrientationValues::kMagInclination,
//       ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
//   sensor_mag_inclination->connect_to(
//       new SKOutputFloat(kSKPathMagInclination, ""));

//   auto* sensor_mag_b_value = new OrientationValues(
//       orientation_sensor, OrientationValues::kMagFieldMagnitude,
//       ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
//   sensor_mag_b_value->connect_to(
//       new SKOutputFloat(kSKPathMagBValue, ""));

//   auto* sensor_mag_b_value_trial = new OrientationValues(
//       orientation_sensor, OrientationValues::kMagFieldMagnitudeTrial,
//       ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
//   sensor_mag_b_value_trial->connect_to(
//       new SKOutputFloat(kSKPathMagBValueTrial, ""));

//  auto* sensor_mag_noise = new OrientationValues(
//       orientation_sensor, OrientationValues::kMagNoiseCovariance,
//       ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
//  auto mag_noise_metadata = std::make_shared<SKMetadata>("unitless", "Magnetic Noise / Interference");
//  auto mag_noise_output = std::make_shared<SKOutput<float>>(
//      kSKPathMagNoise,   // Signal K path
//      "",  // configuration path
//      mag_noise_metadata
//  );
//  sensor_mag_noise->connect_to(mag_noise_output);

  /* This report is a consolidation of all the above magnetic cal
   * values and will need a custom instrument to display.
   */
//   auto* sensor_mag_cal = new MagCalValues(
//       orientation_sensor, ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
//   sensor_mag_cal->connect_to(
//       new SKOutputMagCal(kSKPathMagCalValues, ""));

 /**
   * Following section monitors a physical switch that, when pressed,
   * saves the sensor's current magnetic calibration to non-volatile
   * memory. Comment/uncomment the following as needed.
   *
   * Monitor a button for CHANGEs in state, debounced by kDebounceDelay.
   * No web interface path is supplied, so interval won't be adjustable.
   * Note INPUT_PULLUP may need to change depending on how button is wired.
   */
  /*  TODO - replace with latest SensESP button functions
  auto* button_watcher = new DigitalInputChange(
      PIN_SWITCH_CAL_SAVE, INPUT_PULLUP, CHANGE, "");
  // Create a debounce transform, also with no web interface.
  const int kDebounceDelay = 350; // only react to pushes >350 ms
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
  */
 /**
   * End of physical switch section.
   */

  /** The following section reports turn, roll and pitch rates.
   */
//  auto* sensor_turn_rate = new OrientationValues(
//        orientation_sensor, OrientationValues::kRateOfTurn,
//        ORIENTATION_REPORTING_INTERVAL_MS * 4, kConfigPathTurnRate);
//  auto turn_rate_metadata = std::make_shared<SKMetadata>("°/s", "Turn Rate reported by Compass");
//  auto turn_rate_output = std::make_shared<SKOutput<float>>(
//    kSKPathTurnRate,   // Signal K path
//    kConfigPathTurnRate_SK,  // configuration path
//    turn_rate_metadata
//    );
//  sensor_turn_rate->connect_to(turn_rate_output);

//     auto* sensor_roll_rate = new OrientationValues(
//         orientation_sensor, OrientationValues::kRateOfRoll,
//         ORIENTATION_REPORTING_INTERVAL_MS, kConfigPathRollRate);
//     sensor_roll_rate->connect_to(
//         new SKOutputFloat(kSKPathRollRate, kConfigPathRollRate_SK, metadata_rate_of_roll));

//     auto* sensor_pitch_rate = new OrientationValues(
//         orientation_sensor, OrientationValues::kRateOfPitch,
//         ORIENTATION_REPORTING_INTERVAL_MS, kConfigPathPitchRate);
//     sensor_pitch_rate->connect_to(
//         new SKOutputFloat(kSKPathPitchRate, kConfigPathPitchRate_SK, metadata_rate_of_pitch));

  /* Send the X acceleration as a single value.
   * TODO - it makes sense to send all three accel values (XYZ) in
   * one SK message. The needed data structure is not defined in
   * SensESP. It can be defined in signalk_output.h (as was done for Attitude)
   */
//     auto* sensor_accel_x = new OrientationValues(
//         orientation_sensor, OrientationValues::kAccelerationX,
//         ORIENTATION_REPORTING_INTERVAL_MS, kConfigPathAccelXYZ);
//     sensor_accel_x->connect_to(
//         new SKOutputFloat(kSKPathAccel, kConfigPathAccelXYZ_SK, metadata_accel));

  /** Send Temperature as measured by the orientation sensor.
   * Depending on mounting and enclosure, it may be close to ambient.
   */
  auto* sensor_temperature = new OrientationValues(
      orientation_sensor, OrientationValues::kTemperature);

  auto temperature = std::make_shared<RepeatSensor<float>>(
      10000, //update output value every 10000ms
      [sensor_temperature]() { return sensor_temperature->ReportValue(); });

  // Need to provide metadata, as Temperature related values are not defined in the Signal K spec.
  auto temperature_metadata = std::make_shared<SKMetadata>();
  temperature_metadata->units_ = "K";   
  temperature_metadata->description_ = "Temperature of eCompass IC";
  temperature_metadata->display_name_ = "eCompass Temperature";
  temperature_metadata->short_name_ = "Comp. Temp";

  auto temperature_output = std::make_shared<SKOutput<float>>(
      kSKPathTemperature,   // Signal K path
      "",  // configuration path
      temperature_metadata
  );

  ConfigItem(temperature_output)
      ->set_title("Compass Temperature")
      ->set_description("Internal temperature of eCompass")
      ->set_sort_order(206);

  temperature
        /* Temperature readings are passed through a linear transform
         * to allow for calibration/linearization via web interface. Other
         * transforms are available. Ensure you #include the appropriate file(s).
         */
    ->connect_to(new Linear(1.0, 0.0,kConfigPathTemperatureCal))
    ->connect_to(temperature_output);

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

 
  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
