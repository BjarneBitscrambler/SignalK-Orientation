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
 * Mandatory SensESP headers
 */
#include "sensesp.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

/**
 * If using a mechanical button-press to save Magnetic Calibration, then include
 * digital_input.h and debounce.h
 */
#include "sensesp/sensors/digital_input.h"
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
 * If outputting Attitude values (combined roll, pitch, yaw), then include
 * 
 */
#include "sensesp/signalk/signalk_types.h"

/**
 * If using the Temperature report, then include the linear transform
 * for calibrating the temperature readings.
 */
#include "sensesp/transforms/linear.h"

/**
 * Mandatory Orientation library headers
 */
#include "orientation_sensor.h"
#include "signalk_orientation.h"


// Sensor hardware details: I2C addresses and pins       
#define BOARD_ACCEL_MAG_I2C_ADDR    (0x1F) ///< I2C address on Adafruit breakout board
#define BOARD_GYRO_I2C_ADDR         (0x21) ///< I2C address on Adafruit breakout board
#define PIN_I2C_SDA (23)          // Adjust to your board. A value of -1
#define PIN_I2C_SCL (25)          //   will use default Arduino pins.
#define PIN_SWITCH_CAL_SAVE (32)  // Optional switch attached to this pin saves magnetic calibration
#define SWITCH_ACTIVE_STATE (0)   // Input is LOW when Switch is pushed

// How often orientation parameters are published via Signal K message
// If a report interval is saved for a particular sensor path (via the web
// interface), that overrides the following #define for that report.
#define ORIENTATION_REPORTING_INTERVAL_MS (100)

using namespace sensesp;

/** Producing Magnetic Headings from Compass Readings requires Deviation correction
 * DeviationInterpolator allows one to enter a custom Deviation table. It can be
 * hard-coded below, and updated using SensESP's web interface
 */
class DeviationInterpolator : public CurveInterpolator {
 public:
  DeviationInterpolator(String config_path = "")
      : CurveInterpolator( NULL, config_path) {
    /** Populate a lookup table to translate compass values to magnetic heading.
     * The values should be edited to suit your own actual Deviation table,
     * either here by calling add_sample(), or using the SensESP web config
     * page. Note that values entered via the web configuration will be
     * overwritten by calls to add_sample() in this constructor anytime the ESP
     * device reboots. So if you want the web-entered values to persist, then
     * comment out the below initialization calls.
     *
     * Values are in radians.

     * clear_samples();
     * add_sample(CurveInterpolator::Sample(compass reading, magnetic heading));
    */
    //default Deviation table is straight 1:1 conversion
    //   clear_samples();
    //   add_sample(CurveInterpolator::Sample(0.0, 0.0));
    //   add_sample(CurveInterpolator::Sample(6.3, 6.3));  
  }
};


/** setup() performs one-time application initialization for SensESP and Orientation
 */
void setup() {
  //without following, get runtime messages:
  // I (2147) ARDUINO: LEDC attached to pin 2 (channel 0, resolution 8)
  // E (2149) ARDUINO: IO 0 is not set as GPIO. Execute digitalMode(0, OUTPUT) first.
  // E (2149) ARDUINO: IO 2 is not set as GPIO. Execute digitalMode(2, OUTPUT) first.
  // E (2149) ARDUINO: IO 4 is not set as GPIO. Execute digitalMode(4, OUTPUT) first.
  pinMode(0, OUTPUT); 
  pinMode(2, OUTPUT); //this doesn't prevent the GPIO2 complaint.
  pinMode(4, OUTPUT);

  SetupLogging(ESP_LOG_INFO); //inits the Serial port and starts logging output

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
                    // settings. They can also be configured using the web interface.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("10.0.0.27", 3000)
                    //SensESP has several builtin sensors, e.g. freemem, uptime, IP address
                    //  Optionally enable them here to output their values in SK reports.
                    ->enable_uptime_sensor()
                    ->enable_ip_address_sensor()
                    ->enable_free_mem_sensor()
                    ->enable_system_hz_sensor()
                    //->enable_wifi_signal_sensor()
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
   * Default instruments expect these paths for heading and attitude.
   * You can override them, but will then need to define your
   * own instruments to display the data.
   */
  const char* kSKPathHeadingCompass  = "navigation.headingCompass";
  const char* kSKPathHeadingMagnetic = "navigation.headingMagnetic";
  const char* kSKPathAttitude        = "navigation.attitude";
  /**
   * If you want other parameters in addition to heading, pitch, and roll,
   * uncomment the appropriate SKpath(s) from the following.
   * Signal K v1.7 does not prescribe paths for roll rate and pitch rate
   * so these are provided using the same pattern as for rateOfTurn.
   */
  const char* kSKPathTurnRate    = "navigation.rateOfTurn";
  const char* kSKPathRollRate    = "navigation.rateOfRoll";
  const char* kSKPathPitchRate   = "navigation.rateOfPitch";
  /**
   * Signal K v1.7 says path for temperature can include zone.
   * Replace ecompass with a different zone if desired.
   */
  const char* kSKPathTemperature =
                 "environment.inside.ecompass.temperature";
  /**
   * Signal K v1.7 does not describe a path for acceleration.
   */
  const char* kSKPathAccelX       = "sensors.accelerometer.accel_x"; 
  const char* kSKPathAccelY       = "sensors.accelerometer.accel_y"; 
  const char* kSKPathAccelZ       = "sensors.accelerometer.accel_z"; 
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
  const char* kSKPathMagFit         = "orientation.calibration.magfit";
  const char* kSKPathMagFitTrial    = "orientation.calibration.magfittrial";
  const char* kSKPathMagSolver      = "orientation.calibration.magsolver";
  const char* kSKPathMagInclination = "orientation.calibration.maginclination";
  const char* kSKPathMagBValue      = "orientation.calibration.magmagnitude";
  const char* kSKPathMagBValueTrial = "orientation.calibration.magmagnitudetrial";
  const char* kSKPathMagNoise       = "orientation.calibration.magnoise";
  const char* kSKPathMagCalValues   = "orientation.calibration.magvalues";

  /**
   * The "Configuration path" is combined with "/config" to formulate a URL
   * used by the RESTful API for retrieving or setting configuration data.
   * It is ALSO used to specify a path to the file system
   * where configuration data is saved on the MCU board. If used, it should
   * ALWAYS start with a forward slash. 
   * The max length for a config path is 32 characters, due to a limitation
   * in the SPIFFS filesystem.
   * If the config path is left blank,
   * that indicates this sensor or transform does not have any
   * configuration to save, or that you're not interested in doing
   * run-time configuration.
   * 
   * Note the hardware sensor itself has no run-time configurable items.
   * 
   * The config path for each sensor, if used, is defined in the code region
   * where the sensor itself is defined. If used, the configurable items can
   * be found in the SensESP device's web UI under the Configuration tab.
   * 
   */
  const char* kConfigPathNone = ""; 

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
   * Save_Mag_Cal entry in the sensor web interface (TODO), and/or
   * enable and use the optional hardware switch mentioned later in this code.
   * A calibration remains valid until the sensor's magnetic environment
   * changes.
   */
  auto* orientation_sensor = new OrientationSensor(
      PIN_I2C_SDA, PIN_I2C_SCL, BOARD_ACCEL_MAG_I2C_ADDR, BOARD_GYRO_I2C_ADDR);
  const int fusionIntervalMs = 1000 / orientation_sensor->GetFusionRateHz();
  event_loop()->onRepeat( fusionIntervalMs,
                          [orientation_sensor]() { orientation_sensor->ReadAndProcessSensors(); }
                        );
  /**
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

  /**
   * Enable Compass Heading and Magnetic Heading outputs. The difference
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
   * are accepted by the standard SensESP Curve Interpolator; if you need more,
   * then up to at least 37 pairs are suported by the source code at
   * https://github.com/BjarneBitscrambler/SensESP.git#IncreaseCurveIntPoints
   * The relevant necessary changes are in http.cpp, configurable.cpp, and
   * curveinterpolator.cpp. Note that this modification is based on version
   * 1.0 of SensESP; initial testing with SensESP v2 indicates that the
   * limitation has been removed and the modified source code is no longer
   * needed.
   */

  // Create sensor for compass heading output     
  auto* sensor_heading = new OrientationValues(
      orientation_sensor, OrientationValues::kCompassHeading);
  auto compass_heading = std::make_shared<RepeatSensor<float>>(
      ORIENTATION_REPORTING_INTERVAL_MS,
      [sensor_heading]() { return sensor_heading->ReportValue(); }
    );
  /* RepeatSensor() is not defined as a configurable class, so we can't
     use the web UI to update the reporting rate.  TODO - expand RepeatSensor
     class definition.
   ConfigItem(compass_heading)
    ->set_title("Heading Report Rate")
    ->set_description("interval (ms) between Compass and Magnetic Heading outputs")
    ->set_sort_order(200);
    */
  auto compass_sk_output = std::make_shared<SKOutput<float>>(
      kSKPathHeadingCompass,    // Signal K path
      kConfigPathNone           // configuration path
      );

  /** Set up output for the Magnetic Heading, using the Compass Heading as an input
   *  and passing through a CurveInterpolator transform (class DeviationInterpolator
   *  was created to allow hard-coding of the interpolation values, or you can use
   *  the web UI to enter/edit them.
   */
  auto magneticheading_sk_output = std::make_shared<SKOutput<float>>(
      kSKPathHeadingMagnetic,   // Signal K path
      kConfigPathNone           // configuration path
                                // metadata
      );
  /** Correct for Deviation. 
  * CurveInterpolator applies deviation corrections that can be specified
  * at points within the [0..2Pi] range, and interpolated for points 
  * between the specified ones. Lets you define a Deviation Table.
  * Use the SensESP server's Configuration interface to enter a Compass Deviation table,
  * or hard-code it in the DeviationInterpolator constructor (near top of this file)
  * when using web UI to enter/edit deviation values, they are in JSON format
  * [{"input":float,"output":float},...]
  */
  const char* kConfigPathDeviation = "/sensors/hdg/deviation";
  auto* deviationInterpolator = new DeviationInterpolator(kConfigPathDeviation);
  ConfigItem(deviationInterpolator)
    ->set_title("Deviation Table")
    ->set_description("Interpolation Values")
    ->set_sort_order(1001);

  //Add an AngleCorrection transform, to adjust for any mounting offsets
  //  Pi/2 rotation in this example
  const char* kConfigPathHeadingOffset = "/sensors/hdg/offset";
  auto* mountingOffset = new AngleCorrection((PI/2.0), 0.0, kConfigPathHeadingOffset);
  ConfigItem(mountingOffset)
    ->set_title("Mounting Offset")
    ->set_description("Enter any adjustment to be applied to all headings (e.g. from mounting offsets)")
    ->set_sort_order(400);

  // Connect compass output to Signal K transforms and outputs
  compass_heading
    ->connect_to(mountingOffset)
    ->connect_to(compass_sk_output)
    ->connect_to(deviationInterpolator)
    ->connect_to(new AngleCorrection(0.0, 0.0, kConfigPathNone))  // Normalize to [0..2Pi] after interpolation
    ->connect_to(magneticheading_sk_output);
 
  /** Enable Attitude output (yaw, pitch, roll combined into one SK msg).
   * Note that this output does not pass through any transform to correct for
   * Deviations due to e.g. mounting offsets, magnetic anomalies, etc.
   */

  auto* sensor_roll = new OrientationValues(
      orientation_sensor, OrientationValues::kRoll);
  auto* sensor_pitch = new OrientationValues(
      orientation_sensor, OrientationValues::kPitch);
  auto* sensor_yaw = new OrientationValues(
      orientation_sensor, OrientationValues::kYaw);

  auto attitude_sensor = std::make_shared<RepeatSensor<AttitudeVector>>(
      ORIENTATION_REPORTING_INTERVAL_MS,
      [sensor_roll, sensor_pitch, sensor_yaw]() 
      { return AttitudeVector(sensor_roll->ReportValue(),
                              sensor_pitch->ReportValue(), 
                              sensor_yaw->ReportValue()
                            );
      }
    );
  auto attitude_sk_output = std::make_shared<SKOutput<AttitudeVector>>(
      kSKPathAttitude,    // Signal K path
      kConfigPathNone    // configuration path
  );
  attitude_sensor->connect_to(attitude_sk_output);

  /**
   * The following outputs are useful when calibrating. See the wiki at
   * @see https://github.com/BjarneBitscrambler/SignalK-Orientation/wiki
   * for details on how to interpret the values. None are recognized
   * in the Signal K spec, so there is no prescribed SK path that they
   * need to be sent to. Briefly:
   * Sensor Fusion algorithm continually re-evaluates mag cal based on recent
   * readings, and keeps a 'trial' cal to compare with current cal. If trial 
   * cal becomes superior to current cal, it replaces the current cal.
   * MagCalFit: units %. Measures goodness-of-fit using current cal. Lower is better.
   * MagCalFitTrial: same as MagCalFit, except reports on trial calibration.
   * MagSolver: unitless values [0,4,7,10]. Complexity of current fusion algorithm.
   * MagBValue: units uT. Strength of B field based on present readings.
   * MagBValueTrial: units uT. Strength of B field used in current cal.
   * MagNoise: unitless. Deviation of current reading from calibrated geomag sphere.
   *  Lower is better; above 0.00056 indicates current magnetic reading not reliable.
   * MagInclination: units rad. Magnetic field inclination from horizontal.
   * Because there are quite a few parameters, and they are likely only
   * referred to infrequently (i.e. when calibrating, or when magnetic
   * disturbances are suspected), you may want to configure the Signal K
   * instrument panel to display these paths on a secondary screen,
   * separate from the primary navigation screen.
   */

  // Create output for Magnetic Calibration Fit    
  auto* sensor_magcalfit = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalFitInUse);
  auto magcalfit = std::make_shared<RepeatSensor<float>>(
      4000, //update output value every 4000ms
      [sensor_magcalfit]() { return sensor_magcalfit->ReportValue(); });
  // Need to provide metadata, as Mag Cal related values are not defined in the Signal K spec.
  auto magcalfit_metadata = std::make_shared<SKMetadata>();
  magcalfit_metadata->units_ = "%";   
  magcalfit_metadata->description_ = "Goodness-of-fit of readings using current Magnetic Calibration";
  magcalfit_metadata->display_name_ = "Magnetic Calibration Fit";
  magcalfit_metadata->short_name_ = "MagCalFit";
  auto magcalfit_output = std::make_shared<SKOutput<float>>(
    kSKPathMagFit,    // Signal K path
    kConfigPathNone,  // configuration path
    magcalfit_metadata
  );
  magcalfit->connect_to(magcalfit_output);

  // Create output for Magnetic Calibration Fit Trial     
  auto* sensor_magcal_candidate = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalFitTrial);
  auto magcaltrial = std::make_shared<RepeatSensor<float>>(
      4000, //update output value every 4000ms
      [sensor_magcal_candidate]() { return sensor_magcal_candidate->ReportValue(); });
  auto magcaltrial_metadata = std::make_shared<SKMetadata>();
  magcaltrial_metadata->units_ = "%";
  magcaltrial_metadata->description_ = "Goodness-of-fit of readings using trial Magnetic Calibration";
  magcaltrial_metadata->display_name_ = "Magnetic Calibration Fit Trial";
  magcaltrial_metadata->short_name_ = "MagCalFitTrial";
  auto magcaltrial_output = std::make_shared<SKOutput<float>>(
    kSKPathMagFitTrial,   // Signal K path
    kConfigPathNone,  // configuration path
    magcaltrial_metadata
  );
  magcaltrial->connect_to(magcaltrial_output);

  // Create output for kSKPathMagSolver
  auto* sensor_cal_order = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalAlgorithmSolver);
  auto magcalorder = std::make_shared<RepeatSensor<float>>(
      4000, //update output value every 4000ms
      [sensor_cal_order]() { return sensor_cal_order->ReportValue(); });
  auto cal_solver_metadata = std::make_shared<SKMetadata>();
  cal_solver_metadata->units_ = "[0,4,7,10]";
  cal_solver_metadata->description_ = "Order of calibration algorithm used [0,4,7,10] 10 is best.";
  cal_solver_metadata->display_name_ = "Magnetic Calibration Algorithm Order";
  cal_solver_metadata->short_name_ = "MagCalOrder";
  auto cal_solver_output = std::make_shared<SKOutput<int>>(
    kSKPathMagSolver,  // Signal K path
    "",               // configuration path
    cal_solver_metadata
  );
  magcalorder->connect_to(cal_solver_output);

  // Create output for Magnetic Inclination
  auto* sensor_mag_inclination = new OrientationValues(
       orientation_sensor, OrientationValues::kMagInclination);
  auto maginclination = std::make_shared<RepeatSensor<float>>(
      4000, //update output value every 4000ms
      [sensor_mag_inclination]() { return sensor_mag_inclination->ReportValue(); });
  auto inclination_metadata = std::make_shared<SKMetadata>();
  inclination_metadata->units_ = "rad";
  inclination_metadata->description_ = "Magnetic field inclination from horizontal";
  inclination_metadata->display_name_ = "Magnetic Inclination";
  inclination_metadata->short_name_ = "MagInclination";
  auto inclination_output = std::make_shared<SKOutput<float>>(
    kSKPathMagInclination, // Signal K path
    "",                   // configuration path
    inclination_metadata
  );
  maginclination->connect_to(inclination_output);

  // Create output for Magnetic B Field Strength
  auto* sensor_mag_b_value = new OrientationValues(
      orientation_sensor, OrientationValues::kMagFieldMagnitude);
  auto magbvalue = std::make_shared<RepeatSensor<float>>(
      4000, //update output value every 4000ms
      [sensor_mag_b_value]() { return sensor_mag_b_value->ReportValue(); });
  auto magbvalue_metadata = std::make_shared<SKMetadata>();
  magbvalue_metadata->units_ = "uT";
  magbvalue_metadata->description_ = "Magnetic field strength using current calibration";
  magbvalue_metadata->display_name_ = "Magnetic B Field";
  magbvalue_metadata->short_name_ = "B Field";
  auto magbvalue_output = std::make_shared<SKOutput<float>>(
    kSKPathMagBValue, // Signal K path
    "",               // configuration path
    magbvalue_metadata
  );
  magbvalue->connect_to(magbvalue_output);

  // Create output for Magnetic B Field Strength using Trial Calibration
  auto* sensor_mag_b_trial_value = new OrientationValues(
      orientation_sensor, OrientationValues::kMagFieldMagnitudeTrial);
  auto magbtrialvalue = std::make_shared<RepeatSensor<float>>(
      4000, //update output value every 4000ms
      [sensor_mag_b_trial_value]() { return sensor_mag_b_trial_value->ReportValue(); });
  auto magbtrialvalue_metadata = std::make_shared<SKMetadata>();
  magbtrialvalue_metadata->units_ = "uT";
  magbtrialvalue_metadata->description_ = "Magnetic field strength using trial calibration";
  magbtrialvalue_metadata->display_name_ = "Magnetic B Field Trial";
  magbtrialvalue_metadata->short_name_ = "B Field Trial";
  auto magbtrialvalue_output = std::make_shared<SKOutput<float>>(
    kSKPathMagBValueTrial,  // Signal K path
    "",                     // configuration path
    magbtrialvalue_metadata
  );
  magbtrialvalue->connect_to(magbtrialvalue_output);

  // Create output for Magnetic Noise
  auto* sensor_mag_noise = new OrientationValues(
       orientation_sensor, OrientationValues::kMagNoiseCovariance);
  auto magnoise = std::make_shared<RepeatSensor<float>>(
      4000, //update output value every 4000ms
      [sensor_mag_noise]() { return sensor_mag_noise->ReportValue(); });
  auto mag_noise_metadata = std::make_shared<SKMetadata>();
  mag_noise_metadata->units_ = "unitless";
  mag_noise_metadata->description_ = "Magnetic Noise / Interference";
  mag_noise_metadata->display_name_ = "Magnetic Noise";
  mag_noise_metadata->short_name_ = "Mag Noise"; 
  auto mag_noise_output = std::make_shared<SKOutput<float>>(
      kSKPathMagNoise,   // Signal K path
      "",  // configuration path
      mag_noise_metadata
  );
  magnoise->connect_to(mag_noise_output);

  /** The following section reports turn, roll and pitch rates.
   */
  // Create output for Roll Rate
  auto* sensor_roll_rate = new OrientationValues(
       orientation_sensor, OrientationValues::kRateOfRoll);
  auto roll_rate = std::make_shared<RepeatSensor<float>>(
      200, //update output value every 200ms
      [sensor_roll_rate]() { return sensor_roll_rate->ReportValue(); });
  auto metadata_roll_rate = std::make_shared<SKMetadata>();
  metadata_roll_rate->units_         = "rad/s";
  metadata_roll_rate->description_   = "Rate of Roll about bow-stern axis";
  metadata_roll_rate->display_name_  = "Roll Rate";
  metadata_roll_rate->short_name_    = "Roll Rate"; 
  auto roll_rate_output = std::make_shared<SKOutput<float>>(
      kSKPathRollRate,  // Signal K path
      "",               // configuration path
      metadata_roll_rate
  );
  roll_rate->connect_to(roll_rate_output);
  
 
  // Create output for Pitch Rate
  auto* sensor_pitch_rate = new OrientationValues(
       orientation_sensor, OrientationValues::kRateOfPitch);
  auto pitch_rate = std::make_shared<RepeatSensor<float>>(
      200, //update output value every 200ms
      [sensor_pitch_rate]() { return sensor_pitch_rate->ReportValue(); });
  auto metadata_pitch_rate = std::make_shared<SKMetadata>();
  metadata_pitch_rate->units_         = "rad/s";
  metadata_pitch_rate->description_   = "Rate of Pitch about port-starboard axis";
  metadata_pitch_rate->display_name_  = "Pitch Rate";
  metadata_pitch_rate->short_name_    = "Pitch Rate"; 
  auto pitch_rate_output = std::make_shared<SKOutput<float>>(
      kSKPathPitchRate,  // Signal K path
      "",               // configuration path
      metadata_pitch_rate
  );
  pitch_rate->connect_to(pitch_rate_output);
  
  // Create output for Turn Rate
  auto* sensor_turn_rate = new OrientationValues(
       orientation_sensor, OrientationValues::kRateOfTurn);
  auto turn_rate = std::make_shared<RepeatSensor<float>>(
      200, //update output value every 200ms
      [sensor_turn_rate]() { return sensor_turn_rate->ReportValue(); });
  auto metadata_turn_rate = std::make_shared<SKMetadata>();
  metadata_turn_rate->units_         = "rad/s";
  metadata_turn_rate->description_   = "Rate of Turn about mast-keel axis";
  metadata_turn_rate->display_name_  = "Turn Rate";
  metadata_turn_rate->short_name_    = "Turn Rate"; 
  auto turn_rate_output = std::make_shared<SKOutput<float>>(
      kSKPathTurnRate,  // Signal K path
      "",               // configuration path
      metadata_turn_rate
  );
  turn_rate->connect_to(turn_rate_output);
  

  /**
   *  Relationship of the Acceleration Axes and the terminology:
   * If the sensor is mounted with the X-axis pointing to the bow of the boat
   * and the Y-axis pointing to Port, then Z points up and the normal marine
   * conventions apply. The wiki has details:
   * @see https://github.com/BjarneBitscrambler/SignalK-Orientation/wiki
   *
   * If the sensor is mounted differently, or you prefer an alternate nomenclature,
   * the Get___() methods in sensor_fusion_class.cpp can be adjusted, or the
   * following code can be adjusted.
  */
  /** Send X acceleration as a single value.
   * Y and Z accelerations can be reported using the same format as for X.
   * TODO - it makes sense to send all three accel values (XYZ) in
   * one SK message. The needed data structure is not defined in
   * SensESP. It can be defined in signalk_output.h (as was done for Attitude).
   * A custom Signal K widget to display the combined values will be needed.
   */
  // Create output for X Acceleration
  auto* sensor_accel_X = new OrientationValues(
       orientation_sensor, OrientationValues::kAccelerationX);
  auto accel_X = std::make_shared<RepeatSensor<float>>(
      1000, //update output value every 1000ms
      [sensor_accel_X]() { return sensor_accel_X->ReportValue(); });
  auto metadata_accel_X = std::make_shared<SKMetadata>();
  metadata_accel_X->units_         = "m/s^2";
  metadata_accel_X->description_   = "Acceleration in X axis of eCompass";
  metadata_accel_X->display_name_  = "X Axis Acceleration";
  metadata_accel_X->short_name_    = "Accel X"; 
  auto accel_X_output = std::make_shared<SKOutput<float>>(
      kSKPathAccelX,    // Signal K path
      "",               // configuration path
      metadata_accel_X
  );
  accel_X->connect_to(accel_X_output);


  /** Send Temperature as measured by the orientation sensor.
   * Depending on mounting and enclosure, it may be close to ambient.
   */
  auto* sensor_temperature = new OrientationValues(
      orientation_sensor, OrientationValues::kTemperature);

  auto temperature = std::make_shared<RepeatSensor<float>>(
      1000, //update output value every 1000ms
      [sensor_temperature]() { return sensor_temperature->ReportValue(); });

  /* Temperature readings are passed through a linear transform
  * to allow for calibration/linearization via web interface. Other
  * transforms are available. Ensure you #include the appropriate file(s).
  */
  const char* kConfigPathTemperatureCal = "/sensors/temp/calibrate";
  auto temperatureCal = std::make_shared<Linear>(1.0, 0.0,kConfigPathTemperatureCal);
  ConfigItem(temperatureCal)
      ->set_title("Compass Temperature Calibration")
      ->set_description("Calibration / Linearization of temperature reported by eCompass IC")
      ->set_sort_order(206);

  // Create output, with metadata to indicate source and location of temp data
  auto temperature_metadata = std::make_shared<SKMetadata>();
  temperature_metadata->units_ = "K";   
  temperature_metadata->description_ = "Temperature reported by orientation sensor";
  temperature_metadata->display_name_ = "eCompass Temperature";
  temperature_metadata->short_name_ = "Comp. T";

  auto temperature_output = std::make_shared<SKOutput<float>>(
      kSKPathTemperature,   // Signal K path
      kConfigPathNone,      // configuration path
      temperature_metadata
  );

  temperature
    ->connect_to(temperatureCal)
    ->connect_to(temperature_output);

  /**
   * Following section monitors a physical switch that, when pressed,
   * saves the sensor's current magnetic calibration to non-volatile
   * memory. Comment/uncomment the following as needed.
   *
   * Monitor button for CHANGEs in state, debounced by kDebounceDelay.
   * A web interface path is supplied, so interval is adjustable.
   * INPUT_PULLUP may need to change depending on how button is wired.
   */
  auto* button_watcher = new DigitalInputChange(
      PIN_SWITCH_CAL_SAVE, INPUT_PULLUP, CHANGE, kConfigPathNone);
  // Create a debounce transform
  const int kDebounceDelay = 350; // only react to pushes >350 ms
  const char* kConfigPathDebounceSwitch = "/debounce/delay";
  auto* debounce = new DebounceInt(kDebounceDelay, kConfigPathDebounceSwitch);
  ConfigItem(debounce)
      ->set_title("MagCal Button Debounce")
      ->set_description("Debounce delay (ms) for Magnetic Calibration save button.")
      ->set_sort_order(1000);
  // Define the action taken when button is active and debounce has elapsed.
  // Provide it with the context of orientation_sensor so it can access save fcn.
  auto save_mcal_function = [orientation_sensor](int input) {
    if (input == SWITCH_ACTIVE_STATE) {
      orientation_sensor->sensor_interface_->SaveMagneticCalibration();
      debugI("Magnetic Calibration values saved");
    }
  };
  auto* button_consumer = new LambdaConsumer<int>(save_mcal_function);
  // Connect the button -> debounce -> save magnetic calibration fcn
  button_watcher->connect_to(debounce)->connect_to(button_consumer); 

  
  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
