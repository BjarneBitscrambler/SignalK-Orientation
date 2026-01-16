/** @file orientation_sensor.cpp
 *  @brief Orientation sensor interface to SensESP
 */

#include "orientation_sensor.h"

#include "sensesp.h"

namespace sensesp {
  
/**
 * @brief Constructor sets up the I2C communications to the sensor and
 * initializes the sensor fusion library.
 *
 * @param pin_i2c_sda Pin of SDA line to sensor. Use -1 for Arduino default.
 * @param pin_i2c_scl Pin of SCL line to sensor. Use -1 for Arduino default.
 * @param accel_mag_i2c_addr I2C address of accelerometer/magnetometer IC.
 * @param gyro_i2c_addr I2C address of gyroscope IC.
 * @param config_path RESTful path by which sensor can be configured.
 */
OrientationSensor::OrientationSensor(uint8_t pin_i2c_sda, uint8_t pin_i2c_scl,
                                     uint8_t accel_mag_i2c_addr,
                                     uint8_t gyro_i2c_addr) {
  sensor_interface_ = new SensorFusion();  // create our fusion engine instance

  bool success;
  // init IO subsystem, passing NULLs since we use Signal-K output instead.
  success =
      sensor_interface_->InitializeInputOutputSubsystem(NULL, NULL) &&
      // connect to the sensors.  Accelerometer and magnetometer are in same IC.
      sensor_interface_->InstallSensor(accel_mag_i2c_addr,
                                       SensorType::kMagnetometer) &&
      sensor_interface_->InstallSensor(accel_mag_i2c_addr,
                                       SensorType::kAccelerometer) &&
      // A thermometer (uncalibrated) is available in the
      // accelerometer/magnetometer IC.
      sensor_interface_->InstallSensor(accel_mag_i2c_addr,
                                       SensorType::kThermometer) &&
      sensor_interface_->InstallSensor(gyro_i2c_addr, SensorType::kGyroscope);
  if (!success) {
    debugE("Trouble installing sensors.");
  } else {
    sensor_interface_->Begin(pin_i2c_sda, pin_i2c_scl);
    debugI("Sensors connected & Fusion ready");

    // The Fusion Library, in build.h, defines how fast the ICs generate new
    // orientation data and how fast the fusion algorithm runs, using FUSION_HZ.
    // Usually this rate should be the same as ReadAndProcessSensors() is
    // called.
    const uint32_t kFusionIntervalMs = 1000.0 / FUSION_HZ;
    // Start periodic reads of sensor and running of fusion algorithm.
    /* SensESPv3 changed how React is used
    ReactESP::app->onRepeat(kFusionIntervalMs,
                 [this]() { this->ReadAndProcessSensors(); }); */
    event_loop()->onRepeat(  kFusionIntervalMs,
                             [this]() { this->ReadAndProcessSensors(); }
                          );
  }

}  // end OrientationSensor()

/**
 * @brief Read the Sensors and calculate orientation parameters
 */
void OrientationSensor::ReadAndProcessSensors(void) {
  sensor_interface_->ReadSensors();
  sensor_interface_->RunFusion();

}  // end ReadAndProcessSensors()

/**
 * @brief Constructor sets up the frequency of output and the Signal K path.
 *
 * @param orientation_sensor Pointer to the physical sensor's interface
 * @param report_interval_ms Interval between output reports
 * @param config_path RESTful path by which reporting frequency can be
 * configured.
 */
AttitudeValues::AttitudeValues(OrientationSensor* orientation_sensor,
                               uint report_interval_ms,
                              String config_path)
    : Sensor(config_path),
      orientation_sensor_{orientation_sensor},
      report_interval_ms_{report_interval_ms} {
  //SensESPv3 changed how Config is done  load_configuration();
      save_mag_cal_ = 0;
      event_loop()->onRepeat(  report_interval_ms_,
                             [this]() { this->Update(); }
                          );

}  // end AttitudeValues()

/**
 * @brief Starts periodic output of Attitude parameters.
 *
 * The start() function is inherited from sensesp::Sensor, and is
 * automatically called when the SensESP app starts.
 */
/*void AttitudeValues::start() {
  ReactESP::app->onRepeat(report_interval_ms_, [this]() { this->Update(); });
}
*/
/**
 * @brief Provides one Attitude reading from the orientation sensor.
 *
 * Readings are obtained using the sensor fusion library's Get_() methods
 * and assigned to the output variable that passes data from Producers
 * to Consumers. Consumers of the attitude data are then informed
 * by the call to notify(). If data are not valid (e.g. sensor not
 * functioning), a struct member is set to false so when the Signal K
 * message contents are assembled by as_signalk(),they can reflect that. 
 */
void AttitudeValues::Update() {
  //check whether magnetic calibration has been requested to be saved or deleted
  if( 1 == save_mag_cal_ ) {
    orientation_sensor_->sensor_interface_->InjectCommand("SVMC");
  }else if( -1 == save_mag_cal_ ) {
    orientation_sensor_->sensor_interface_->InjectCommand("ERMC");
  }
  save_mag_cal_ = 0;  // set flag back to zero so we don't repeat save/delete
  attitude_.is_data_valid =
      orientation_sensor_->sensor_interface_->IsDataValid();
  attitude_.yaw = orientation_sensor_->sensor_interface_->GetHeadingRadians();
  attitude_.roll =
      orientation_sensor_->sensor_interface_->GetRollRadians();
  attitude_.pitch =
      orientation_sensor_->sensor_interface_->GetPitchRadians();

  //output = attitude_;  //output is done differently in SensESPv3  as_signalk_json()
  //notify();
  this->emit(attitude_); //in valueproducer.h  Sets output and calls notify();

}  // end Update()

/**
 * @brief Define the format for the Orientation value producers.
 *
 * This format is common to the single orientation parameter producers
 * (OrientationValues objects) and the attitude parameter producers
 * (AttitudeValues objects).
 */
static const char SCHEMA[] PROGMEM = R"###({
    "type": "object",
    "properties": {
        "report_interval": { 
          "title": "Report Interval", 
          "type": "number", 
          "description": "Milliseconds between outputs of this parameter" 
        },
        "save_mag_cal": { 
          "title": "Save Magnetic Cal", 
          "type": "number", 
          "description": "Set to 1 to save current magnetic calibration" 
        }
    }
  })###";

/**
 * @brief Get the current sensor configuration and place it in a JSON
 * object that can then be stored in non-volatile memory.
 * 
 * @param doc JSON object to contain the configuration parameters
 * to be updated.
 */
/* changed for SensESPv3 TODO - look into replacement */
/*void AttitudeValues::get_configuration(JsonObject& doc) {
  doc["report_interval"] = report_interval_ms_;
  doc["save_mag_cal"] = save_mag_cal_;
}  // end get_configuration() */

/**
 * @brief Fetch the JSON format used for holding the configuration.
 */
/* changed for SensESPv3 TODO - look into replacement */
//  String AttitudeValues::get_config_schema() { return FPSTR(SCHEMA); }

/**
 * @brief Use the values stored in JSON object config to update
 * the appropriate member variables.
 *
 * @param config JSON object containing the configuration parameters
 * to be updated.
 * @return True if successful; False if a parameter could not be found.
 */
/* changed for SensESPv3 TODO - look into replacement */
/*bool AttitudeValues::set_configuration(const JsonObject& config) {
  String expected[] = {"report_interval", "save_mag_cal"};
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }
  report_interval_ms_ = config["report_interval"];
  save_mag_cal_ = config["save_mag_cal"];
  return true;
}  // end set_configuration()
*/

/**
 * @brief Constructor sets up the frequency of output and the Signal K path.
 *
 * @param orientation_sensor Pointer to the physical sensor's interface
 * @param report_interval_ms Interval between output reports
 * @param config_path RESTful path by which reporting frequency can be
 * configured.
 */
MagCalValues::MagCalValues(OrientationSensor* orientation_sensor,
                          uint report_interval_ms,
                          String config_path)
    : Sensor(config_path),
      orientation_sensor_{orientation_sensor},
      report_interval_ms_{report_interval_ms} {
  //removed for SensESPv3 load_configuration();
    event_loop()->onRepeat(  report_interval_ms_,
                             [this]() { this->Update(); }
    );

}  // end MagCalValues()

/**
 * @brief Starts periodic output of MagCal parameters.
 *
 * The start() function is inherited from sensesp::Sensor, and is
 * automatically called when the SensESP app starts.
 */
/*void MagCalValues::start() {
  ReactESP::app->onRepeat(report_interval_ms_, [this]() { this->Update(); });
}
*/


/**
 * @brief Provides one MagCal reading from the orientation sensor.
 *
 * Readings are obtained using the sensor fusion library's Get_() methods
 * and assigned to the output variable that passes data from Producers
 * to Consumers. Consumers of the magcal data are then informed
 * by the call to notify(). If data are not valid (e.g. sensor not
 * functioning), a struct member is set to false so when the Signal K
 * message contents are assembled by as_signalk(),they can reflect that. 
 */
void MagCalValues::Update() {
  mag_cal_.is_data_valid =
      orientation_sensor_->sensor_interface_->IsDataValid();
  mag_cal_.cal_fit_error = orientation_sensor_->sensor_interface_->GetMagneticFitError() / 100.0;
  mag_cal_.cal_fit_error_trial = orientation_sensor_->sensor_interface_->GetMagneticFitErrorTrial() / 100.0;
  mag_cal_.mag_field_magnitude = orientation_sensor_->sensor_interface_->GetMagneticBMag();
  mag_cal_.mag_field_magnitude_trial = orientation_sensor_->sensor_interface_->GetMagneticBMagTrial();
  mag_cal_.mag_noise_covariance = orientation_sensor_->sensor_interface_->GetMagneticNoiseCovariance();
  mag_cal_.mag_solver = orientation_sensor_->sensor_interface_->GetMagneticCalSolver();
  mag_cal_.magnetic_inclination = orientation_sensor_->sensor_interface_->GetMagneticInclinationRad();

  //output = mag_cal_;  //TODO figure out hoiw the outputs are managed
  //notify();
  this->emit(mag_cal_); //in valueproducer.h  Sets output and calls notify();

}  // end Update()

/**
 * @brief Define the format for the MagCal value producer.
 *
 */
static const char SCHEMA_MAGCAL[] PROGMEM = R"###({
    "type": "object",
    "properties": {
        "report_interval": { 
          "title": "Report Interval", 
          "type": "number", 
          "description": "Milliseconds between outputs of this parameter" 
        }
    }
  })###";

/**
 * @brief Get the current sensor configuration and place it in a JSON
 * object that can then be stored in non-volatile memory.
 * 
 * @param doc JSON object to contain the configuration parameters
 * to be updated.
 */
/*void MagCalValues::get_configuration(JsonObject& doc) {
  doc["report_interval"] = report_interval_ms_;
}  // end get_configuration()
*/

/**
 * @brief Fetch the JSON format used for holding the configuration.
 */
//  String MagCalValues::get_config_schema() { return FPSTR(SCHEMA_MAGCAL); }

/**
 * @brief Use the values stored in JSON object config to update
 * the appropriate member variables.
 *
 * @param config JSON object containing the configuration parameters
 * to be updated.
 * @return True if successful; False if a parameter could not be found.
 */
/*bool MagCalValues::set_configuration(const JsonObject& config) {
  String expected[] = {"report_interval"};
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }
  report_interval_ms_ = config["report_interval"];
  return true;
}  // end set_configuration()
*/

/**
 * @brief Constructor sets up the frequency of output and the Signal K path.
 *
 * @param orientation_sensor Pointer to the physical sensor's interface
 * @param val_type The type of orientation parameter to be sent
 * @param report_interval_ms Interval between output reports
 * @param config_path RESTful path by which reporting frequency can be
 * configured.
 */

/**
 * @brief Starts periodic output of orientation parameter.
 *
 * The start() function is inherited from sensesp::Sensor, and is
 * automatically called when the SensESP app starts.
 */
/*void OrientationValues::start() {
  ReactESP::app->onRepeat(report_interval_ms_, [this]() { this->Update(); });
}
*/



/**
 * @brief Get the current sensor configuration and place it in a JSON
 * object that can then be stored in non-volatile memory.
 *
 * @param doc JSON object to contain the configuration parameters
 * to be updated.
 */
/*void OrientationValues::get_configuration(JsonObject& doc) {
  doc["report_interval"] = report_interval_ms_;
  doc["save_mag_cal"] = save_mag_cal_;
}  // end get_configuration()
*/
/**
 * @brief Fetch the JSON format used for holding the configuration.
 */
//String OrientationValues::get_config_schema() { return FPSTR(SCHEMA); }

/**
 * @brief Use the values stored in JSON object config to update
 * the appropriate member variables.
 *
 * @param config JSON object containing the configuration parameters
 * to be updated.
 * @return True if successful; False if a parameter could not be found.
 */
/*bool OrientationValues::set_configuration(const JsonObject& config) {
  String expected[] = {"report_interval", "save_mag_cal"};
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }
  report_interval_ms_ = config["report_interval"];
  save_mag_cal_ = config["save_mag_cal"];
  return true;
}
*/
OrientationValues::OrientationValues(OrientationSensor* orientation_sensor,
                                     OrientationValType val_type,
                                     int report_interval_ms)
    {
      orientation_sensor_ = orientation_sensor;
      value_type_ = val_type;
      report_interval_ms_ = report_interval_ms; 
      throttlePrint_ = 0;
  //load_configuration();

  save_mag_cal_ = 0;

}  // end OrientationValues()
float OrientationValues::ReportValue() {
  //check whether magnetic calibration has been requested to be saved or deleted
  if( 1 == save_mag_cal_ ) {
    orientation_sensor_->sensor_interface_->InjectCommand("SVMC");
  }else if( -1 == save_mag_cal_ ) {
    orientation_sensor_->sensor_interface_->InjectCommand("ERMC");
  }
  save_mag_cal_ = 0;  // set flag back to zero so we don't repeat save/delete
  //check which type of parameter is requested, and pass it on
  
  float output = 0.0;

  switch (value_type_) {
    case (kCompassHeading):
      output = orientation_sensor_->sensor_interface_->GetHeadingRadians();
        throttlePrint_++;
        if( (throttlePrint_ % 50) == 0 )
        { debugI("Angle2: %f", output);
        }
        break;
    case (kRoll):
      output = orientation_sensor_->sensor_interface_->GetRollRadians();
      break;
    case (kPitch):
      output = orientation_sensor_->sensor_interface_->GetPitchRadians();
      break;
    case (kAccelerationX):
      output = orientation_sensor_->sensor_interface_->GetAccelXMPerSS();
      break;
    case (kAccelerationY):
      output = orientation_sensor_->sensor_interface_->GetAccelYMPerSS();
      break;
    case (kAccelerationZ):
      output = orientation_sensor_->sensor_interface_->GetAccelZMPerSS();
      break;
    case (kRateOfTurn):
      output = orientation_sensor_->sensor_interface_->GetTurnRateRadPerS();
      break;
    case (kRateOfPitch):
      output = orientation_sensor_->sensor_interface_->GetPitchRateRadPerS();
      break;
    case (kRateOfRoll):
      output = orientation_sensor_->sensor_interface_->GetRollRateRadPerS();
      break;
    case (kTemperature):
      output = orientation_sensor_->sensor_interface_->GetTemperatureK();
      break;
    case (kMagCalFitInUse):
      output = orientation_sensor_->sensor_interface_->GetMagneticFitError();
      break;
    case (kMagCalFitTrial):
      output = orientation_sensor_->sensor_interface_->GetMagneticFitErrorTrial();
      break;
    case (kMagCalAlgorithmSolver):
      output = orientation_sensor_->sensor_interface_->GetMagneticCalSolver();
      break;
    case (kMagInclination):
      output = orientation_sensor_->sensor_interface_->GetMagneticInclinationRad();
      break;
    case (kMagFieldMagnitude):
      //TODO report in T rather than uT, however need widget to be able to display
      output = orientation_sensor_->sensor_interface_->GetMagneticBMag();
      break;
    case (kMagFieldMagnitudeTrial):
      //TODO report in T rather than uT, however need widget to be able to display
      output = orientation_sensor_->sensor_interface_->GetMagneticBMagTrial();
      break;
    case (kMagNoiseCovariance):
      output = orientation_sensor_->sensor_interface_->GetMagneticNoiseCovariance();
      break;
    default:
      break; 
  }
  return output;
}  // end Update()



} //namespace sensesp