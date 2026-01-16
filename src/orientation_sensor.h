/** @file orientation_sensor.h
 *  @brief Orientation sensor interface to SensESP
 * 
 * Provides Orientation from 9DOF sensor combination (magnetometer,
 * accelerometer, gyroscope) consisting of FXOS8700 + FXAS21002
 */

#ifndef orientation_sensor_H_
#define orientation_sensor_H_

#include "sensor_fusion_class.h"  // for OrientationSensorFusion-ESP library

#include "sensesp/sensors/sensor.h"
#include "signalk_orientation.h"

namespace sensesp {
/**
 * @brief OrientationSensor represents a 9-Degrees-of-Freedom sensor
 * (magnetometer, accelerometer, and gyroscope).
 *
 * This class provides the interface to the SensorFusion library which performs
 * the I2C communication with the sensor and runs the sensor fusion algorithm.
 *
 * A compatible sensor is the NXP FXOS8700 + FXAS21002 combination sensor.
 * This combination sensor is found on products such as the
 * Adafruit #3463 breakout board. The OrientationSensorFusion-ESP
 * library is configured to use this NXP sensor by default, though
 * other sensors can be used by adjusting the library's build.h
 * and board.h files. Calling the public SensorFusion:: methods
 * can be done after you instantiate OrientationSensor, for example by:
 * orientation_sensor->sensor_interface_->GetOrientationQuaternion();
 * The OrientationSensorFusion-ESP library has details:
 * @see https://github.com/BjarneBitscrambler/OrientationSensorFusion-ESP.git
  */
class OrientationSensor {
 public:
  OrientationSensor(uint8_t pin_i2c_sda, uint8_t pin_i2c_scl,
                    uint8_t accel_mag_i2c_addr, uint8_t gyro_i2c_addr);
  SensorFusion* sensor_interface_;  ///< sensor's Fusion Library interface

 private:
  void ReadAndProcessSensors(void);  ///< reads sensor and runs fusion algorithm
};

/**
 * @brief AttitudeValues reads and outputs attitude (yaw,pitch,roll) parameters.
 *
 * The three parameters are stored in an Attitude struct, and sent together
 * in one Signal K message. The units are radians.
 */
//bj removed  public virtual AttitudeProducer, 
class AttitudeValues : public  sensesp::Sensor <Attitude> {
 public:
  AttitudeValues(OrientationSensor* orientation_sensor,
                uint report_interval_ms = 100,
                String config_path = "");
//sensESP v3 removes start. Should start emitting data in constructor I think
  //void start() override final;  ///< starts periodic outputs of Attitude
  OrientationSensor*
      orientation_sensor_;  ///< Pointer to the orientation sensor

 private:
  void Update(void);  ///< fetches current attitude and notifies consumer
  //SensESPv3 requires inheriting from Configurable and explicitly calling ConfigItem()
  //virtual void get_configuration(JsonObject& doc) override;
  //virtual bool set_configuration(const JsonObject& config) override;
  //virtual String get_config_schema() override;
  Attitude attitude_;  ///< struct storing the current yaw,pitch,roll values
  uint report_interval_ms_;  ///< interval between attitude updates to Signal K
  int8_t save_mag_cal_;      ///< Flag for saving current magnetic calibration

};  // end class AttitudeValues

/**
 * @brief MagCalValues reads and outputs magnetic calibration parameters.
 *
 * The parameters are stored in an MagCal struct, and sent together
 * in one Signal K message. They are useful in determining how well
 * the existing magnetic calibration suits the current magnetic
 * environment.
 */
//removed : public MagCalProducer,
class MagCalValues : public sensesp::Sensor <MagCal>{
 public:
  MagCalValues(OrientationSensor* orientation_sensor,
                uint report_interval_ms = 100,
                String config_path = "");
//sensESP v3 removes start(). Presumable have to start in Constructor
  //void start() override final;  ///< starts periodic outputs of MagCal values
  OrientationSensor*
      orientation_sensor_;  ///< Pointer to the orientation sensor

 private:
  void Update(void);  ///< fetches current attitude and notifies consumer
  //SensESPv3 requires inheriting from Configurable and explicitly calling ConfigItem()
  //virtual void get_configuration(JsonObject& doc) override;
  //virtual bool set_configuration(const JsonObject& config) override;
  //virtual String get_config_schema() override;
  MagCal mag_cal_;  ///< struct storing the current magnetic calibration parameters
  uint report_interval_ms_;  ///< interval between attitude updates to Signal K

};  // end class MagCalValues



/**
 * @brief OrientationValues reads and outputs orientation parameters.
 *
 * One parameter is sent per instance of OrientationValues, selected
 * from the list of OrientationValType. The one exception is the
 * attitude (yaw,pitch,roll) which consists of three parameters and
 * is provided by the AttitudeValues class instead of this one.
 * Create new instances in main.cpp for each parameter desired.
 */
//sensESP v2 replaced NumericSensor with various sub-types  class OrientationValues : public NumericSensor {
class OrientationValues  {
 public:
  enum OrientationValType {
    kCompassHeading,      ///< compass heading, also called yaw
    kYaw,                 ///< rotation about the vertical axis
    kPitch,               ///< rotation about the transverse axis
    kRoll,                ///< rotation about the longitudinal axis
    kAttitude,            ///< attitude combines heading, pitch, and roll
    kAccelerationX,       ///< acceleration in the stern-to-bow axis
    kAccelerationY,       ///< acceleration in the starboard-to-port axis
    kAccelerationZ,       ///< acceleration in the down-to-up axis
    kRateOfTurn,          ///< rate of change of compass heading
    kRateOfPitch,         ///< rate of change of pitch
    kRateOfRoll,          ///< rate of change of roll
    kTemperature,         ///< temperature as reported by sensor IC
    kMagCalFitInUse,      ///< fit of currently-used calibration. <3.5 is good.
    kMagCalFitTrial,      ///< fit of candidate calibration. <3.5 is good.
    kMagCalAlgorithmSolver,   ///< cal algorithm order used. [0,4,7,10] 10 is best
    kMagInclination,      ///< geomagnetic inclination based on current readings
    kMagFieldMagnitude,   ///< geomagnetic magnitude of current calibration
    kMagFieldMagnitudeTrial,  ///< geomagnetic magnitude based on current readings
    kMagNoiseCovariance   ///< deviation of current reading from calibrated geomag sphere
  };
  OrientationValues(OrientationSensor* orientation_sensor,
                    OrientationValType value_type = kCompassHeading,
                    int report_interval_ms = 100);
//sensESP v3 removes start()
  //void start() override final;  ///< starts periodic outputs of Attitude
  OrientationSensor*
      orientation_sensor_;  ///< Pointer to the orientation sensor
 float ReportValue(
      void);  ///< fetches current orientation parameter and notifies consumer
 
 private:
  //SensESPv3 requires inheriting from Configurable and explicitly calling ConfigItem()
  //virtual void get_configuration(JsonObject& doc) override;
  //virtual bool set_configuration(const JsonObject& config) override;
  //virtual String get_config_schema() override;
  OrientationValType
      value_type_;  ///< Particular type of orientation parameter supplied
  uint report_interval_ms_;  ///< Interval between data outputs via Signal K
  int8_t save_mag_cal_;      ///< Flag for saving current magnetic calibration
  int throttlePrint_;

};  // end class OrientationValues2


} // namespace sensesp

#endif  // ORIENTATION_SENSOR_H_
