/*******************************************************************************
 * Copyright 2012 Bryan Godbolt, Hasitha Senanayake
 * 
 * This file is part of ANCL Autopilot.
 * 
 *     ANCL Autopilot is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 * 
 *     ANCL Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 * 
 *     You should have received a copy of the GNU General Public License
 *     along with ANCL Autopilot.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

#ifndef HELICOPTER_H
#define HELICOPTER_H

/* STL Headers */
#include <iostream>	// for debugging

/* Boost Headers */
#include <boost/array.hpp>
#include <boost/numeric/ublas/vector.hpp>
namespace blas = boost::numeric::ublas;

/* Project Headers */
#include "servo_switch.h"
#include "RadioCalibration.h"
#include "RCTrans.h"
#include "heli.h"

/**
    \brief This class handles output pulse scaling from normalized values
    based on calibration data from RadioCalibration.

    \author Hasitha Senanayake <senanaya@ualberta.ca>
    \author Bryan Godbolt <godbolt@ece.ualberta.ca>
    \date July 14, 2011: Created class

*/

class Helicopter
{
public:
  static Helicopter* getInstance();

  /** Returns Aileron channel pulse value derived from scaled pulse
		@param norm scaled value of pulse
		@return pulse pulse value from value scaled with respect to Radio calibration end-points.
		*/
  uint16_t setAileron(double norm);
  /** Returns Elevator channel pulse value derived from scaled pulse
		@param norm scaled value of pulse
		@return pulse pulse value from value scaled with respect to Radio calibration end-points.
		*/
  uint16_t setElevator(double norm);
  /** Returns "Throttle" channel pulse value derived from scaled pulse
		@param norm scaled value of pulse
		@return pulse pulse value from value scaled with respect to Radio calibration end-points.
		*/
  uint16_t setThrottle(double norm);
  /** Returns pulse value on channel controlling Rudder, which is derived from scaled pulse
		@param norm scaled value of pulse
		@return pulse pulse value from value scaled with respect to Radio calibration end-points.
		*/
  uint16_t setRudder(double norm);
  /** Returns Gyro channel's pulse value on channel controlling Rudder, which is derived from scaled pulse
		@param norm scaled value of pulse
		@return pulse pulse value from value scaled with respect to Radio calibration end-points.
		*/
  uint16_t setGyro(double norm);
  /** Returns Collective Pitch pulse value derived from scaled pulse
		@param norm scaled value of pulse
		@return pulse pulse value from value scaled with respect to Radio calibration end-points.
		*/
  uint16_t setPitch(double norm);

  /** returns an array of derived pulses (from their scaled values) for channels 1 - 6. */
  boost::array<uint16_t, 6> setScaled(boost::array<double, 6> norm);
  /** @param norm vector of scaled pulse values for all 6 channels
   	   @return pulse vector of de-normalized pulse values for all 6 channels */
  std::vector<uint16_t> setScaled(blas::vector<double> norm) {return setScaled(std::vector<double>(norm.begin(), norm.end()));}
  /** @param norm vector of scaled pulse values for all 6 channels
   	   @return pulse vector of de-normalized pulse values for all 6 channels */
  std::vector<uint16_t> setScaled(std::vector<double> norm);
private:
  /// Singleton Constructor.
  Helicopter();
  /// stores a pointer to the instance of this class.
  static Helicopter* _instance;
  /// Pointer to an instance of RadioCalibration class, to obtain the calibrated set points.
  RadioCalibration *radio_cal_data;
  /// Pointer to an instance of servo_switch to output the channel values.
  servo_switch *out;

  /** Returns a pulse value from a scaled value, with respect to 2 Radio calibration end-points.
      @param norm the scaled pulse value
      @param setpoint an array that stores the calibrated end point pulse values of the Radio
      @return pulse pulse value derived from the scaled value with respect to end points */
  uint16_t norm2pulse(double norm, boost::array<uint16_t, 2> setpoint);
  /** Returns a pulse value from a scaled value, with respect to 3 Radio calibration end-points.
      @param norm the scaled pulse value
      @param setpoint an array that stores the calibrated end point pulse values of the Radio
      @return pulse pulse value derived from the scaled value with respect to end points */
  uint16_t norm2pulse(double norm, boost::array<uint16_t, 3> setpoint);
  /** Returns a pulse value from a scaled value, with respect to 5 Radio calibration end-points.
      @param norm the scaled pulse value
      @param setpoint an array that stores the calibrated end point pulse values of the Radio
      @return pulse pulse value derived from the scaled value with respect to end points */
  uint16_t norm2pulse(double norm, boost::array<uint16_t, 5> setpoint);

  /// List provides index to channel mapping for the Helicopter::getScaledPulses function.
  enum RadioElement
  {
    AILERON = 0,
    ELEVATOR,
    THROTTLE,
    RUDDER,
    GYRO,
    PITCH
  };

};

#endif // HELICOPTER_H
