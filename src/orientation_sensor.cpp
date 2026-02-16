/** @file orientation_sensor.cpp
 *  @brief Orientation sensor interface to SensESP
 */

#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#ifndef TAG
//avoid conflicts if TAG (used by logging fcns to indicate source) is defined elsewhere.
static const char* TAG = "orientation_sensor.cpp";
#endif

#include "esp_log.h"

#include "orientation_sensor.h"

  
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
    //debugE("Trouble installing sensors.");
    // ...
    ESP_LOGE(TAG, "Trouble installing sensors.");
  } else {
    sensor_interface_->Begin(pin_i2c_sda, pin_i2c_scl);
    //debugI("Sensors connected & Fusion ready");
    ESP_LOGI(TAG, "Sensors connected & Fusion ready");

    // The Fusion Library, in build.h, defines how fast the ICs generate new
    // orientation data and how fast the fusion algorithm runs, using FUSION_HZ.
    // Usually this rate should be the same as ReadAndProcessSensors() is
    // called.
    // We rely on the main program using this library to call ReadAndProcessSensors()
    // at the rate given by a call to GetFusionRateHz().  
    // Calls to fetch the Orientation values then should run at not faster than the
    // rate at which ReadAndProcessSensors() is called. e.g. retrieving attitude values
    // at 10 Hz is fine when the fusion rate is 40 Hz, but retrieving attitude values
    // at 50 Hz would result in duplicate values.
  }

}  // end OrientationSensor()

/**
 * @brief Return how fast we read the physical sensor and run the fusion algorithm
 */
int OrientationSensor::GetFusionRateHz(void) {
  return FUSION_HZ;

}  // end ReadAndProcessSensors()

/**
 * @brief Read the Sensors and calculate orientation parameters
 */
void OrientationSensor::ReadAndProcessSensors(void) {
  sensor_interface_->ReadSensors();
  sensor_interface_->RunFusion();

}  // end ReadAndProcessSensors()


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


OrientationValues::OrientationValues(OrientationSensor* orientation_sensor,
                                     OrientationValType val_type)
    {
      orientation_sensor_ = orientation_sensor;
      value_type_ = val_type; 
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
    case (kYaw):
      output = orientation_sensor_->sensor_interface_->GetHeadingRadians();
        throttlePrint_++;
        if( (throttlePrint_ % 50) == 0 )
        { ESP_LOGI(TAG, "Yaw: %f", output);         
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
