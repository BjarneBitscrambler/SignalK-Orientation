/** @file signalk_orientation.h
 *  @brief Vessel orientation data structures definition file
 */

#ifndef _signalk_orientation_H_
#define _signalk_orientation_H_

#include "../.pio/libdeps/esp32dev/SensESP/src/sensesp/system/valueproducer.h"

namespace sensesp {
/**
 * Attitude struct contains the yaw, pitch, and roll values from
 * the orientation sensor-fusion algorithm. Additionally,
 * an out-of-band signal is needed to pass the information from the
 * Producer (sensor) to the Consumer (SignalKOutput) whether the data
 * is valid or not. Using magic numbers can be done (e.g. a negative
 * value for temperature or heading) but needs customization for
 * particular parameters and data types. Rather than special values,
 * for Attitude we add a field to the struct that indicates
 * whether or not the numerical members are valid.
 */
struct Attitude {
  bool is_data_valid;  ///< Indicates whether yaw,pitch,roll data are valid.
  float yaw;           ///< Compass heading of the vessel in radians.
  float pitch;         ///< Rotation about transverse axis in radians. Bow up is
                       ///< positive.
  float roll;  ///< Rotation about longitudinal axis in radians. Starboard roll
               ///< is positive.
};

typedef ValueProducer<Attitude> AttitudeProducer;

/**
 * MagCal struct contains the values relating to magnetic calibration
 * from the orientation sensor-fusion algorithm. These assist the user
 * in deciding whether the in-use magnetic calibration is suitable or
 * whether the current trial calibration is an improvement. The trial
 * calibration is continuously-updated based on recent magnetic 
 * readings.
 * 
 */
struct MagCal {
  bool is_data_valid;          ///< Indicates whether data are valid.
  float magnetic_inclination;  ///< Magnetic field inclination from horizontal
                               ///< in radians.
  float cal_fit_error;  ///< error in current calibration's fit, expressed as
                        ///< percent ratio
  float cal_fit_error_trial;  ///< error in trial calibration's fit, expressed
                              ///< as percent ratio
  float mag_field_magnitude;  ///< geomagnetic field magnitude used in current
                              ///< calibration, in T
  float mag_field_magnitude_trial;  ///< geomagnetic field magnitude based on
                                    ///< recent readings, in T
  float mag_noise_covariance;       ///< covariance of magnetic noise of current
                                    ///< reading  TODO check units
  int mag_solver;  ///< solver used for current magnetic calibration. Unitless,
                   ///< in set [0,4,7,10]
};

typedef ValueProducer<MagCal> MagCalProducer;

} // namespace sensesp

#endif  // _signalk_orientation_H_
