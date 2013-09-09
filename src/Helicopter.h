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
#include <string>
#include <vector>

/* Boost Headers */
#include <boost/array.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/banded.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/thread.hpp>

/* Project Headers */
#include "servo_switch.h"
#include "RadioCalibration.h"
#include "RCTrans.h"
#include "heli.h"
#include "Parameter.h"
#include "Debug.h"

/**
    \brief This class handles output pulse scaling from normalized values
    based on calibration data from RadioCalibration.

    \author Bryan Godbolt <godbolt@ece.ualberta.ca>
    \author Hasitha Senanayake <senanaya@ualberta.ca>
    \date July 14, 2011: Created class
    @date February 2012: Refactored to use boost ublas types
    @date September 27, 2012: Added physical parameters
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

  /// get the helicopter's mass (kg)
  inline double get_mass() const {boost::mutex::scoped_lock(mass_lock); return mass;}
  /// get gravity (m/s^2)
  inline double get_gravity() const {return gravity;}
  /// return the main rotor hub offset
  inline blas::vector<double> get_main_hub_offset() const {boost::mutex::scoped_lock(main_hub_offset_lock); return main_hub_offset;}
  /// return the tail rotor hub offset
  inline blas::vector<double> get_tail_hub_offset() const {boost::mutex::scoped_lock(tail_hub_offset_lock); return tail_hub_offset;}
  /// return the inertia matrix
  inline blas::banded_matrix<double> get_inertia() const {boost::mutex::scoped_lock(inertia_lock); return inertia;}

  /// get a list of helicopter parameters
  std::vector<Parameter> getParameters();
  /// set a parameter value
  void setParameter(Parameter p);

  static const std::string PARAM_MASS;

  static const std::string PARAM_MAIN_OFFSET_X;
  static const std::string PARAM_MAIN_OFFSET_Y;
  static const std::string PARAM_MAIN_OFFSET_Z;

  static const std::string PARAM_TAIL_OFFSET_X;
  static const std::string PARAM_TAIL_OFFSET_Y;
  static const std::string PARAM_TAIL_OFFSET_Z;

  static const std::string PARAM_INERTIA_X;
  static const std::string PARAM_INERTIA_Y;
  static const std::string PARAM_INERTIA_Z;

  double get_main_collective() const;

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

  /// helicopter mass (kg)
  double mass;
  /// serialize access to mass
  mutable boost::mutex mass_lock;
  /// set the mass
  inline void set_mass(const double mass) {{boost::mutex::scoped_lock(mass_lock); this->mass = mass;} message() << "Mass set to " << mass;}

  /// acceleration due to gravity (m/s^2)
  const double gravity;

  /// main rotor hub offset from com (m)
  blas::vector<double> main_hub_offset;
  /// serialize access to main_hub_offset
  mutable boost::mutex main_hub_offset_lock;
  /// set the main rotor hub offset
  inline void set_main_hub_offset(const blas::vector<double>& main_hub_offset) {boost::mutex::scoped_lock(main_hub_offset_lock); this->main_hub_offset = main_hub_offset;}
  /// set the body x main_hub_offset
  inline void set_main_hub_offset_x(const double& main_hub_offset_x) {{boost::mutex::scoped_lock(main_hub_offset_lock); main_hub_offset(0) = main_hub_offset_x;} message() << "Main Hub Offset x set to " << main_hub_offset_x;}
  /// set the body y main_hub_offset
  inline void set_main_hub_offset_y(const double& main_hub_offset_y) {{boost::mutex::scoped_lock(main_hub_offset_lock); main_hub_offset(1) = main_hub_offset_y;} message() << "Main Hub Offset y set to " << main_hub_offset_y;}
  /// set the body z main_hub_offset
  inline void set_main_hub_offset_z(const double& main_hub_offset_z) {{boost::mutex::scoped_lock(main_hub_offset_lock); main_hub_offset(2) = main_hub_offset_z;} message() << "Main Hub Offset z set to " << main_hub_offset_z;}

  /// tail rotor hub offset from com (m)
  blas::vector<double> tail_hub_offset;
  /// serialize access to tail_hub_offset
  mutable boost::mutex tail_hub_offset_lock;
  /// set the tail rotor hub offset
  inline void set_tail_hub_offset(const blas::vector<double>& tail_hub_offset) {boost::mutex::scoped_lock(tail_hub_offset_lock); this->tail_hub_offset = tail_hub_offset;}
  /// set the body x tail_hub_offset
  inline void set_tail_hub_offset_x(const double& tail_hub_offset_x) {{boost::mutex::scoped_lock(tail_hub_offset_lock); tail_hub_offset(0) = tail_hub_offset_x;} message() << "Tail Hub Offset x set to " << tail_hub_offset_x;}
  /// set the body y tail_hub_offset
  inline void set_tail_hub_offset_y(const double& tail_hub_offset_y) {{boost::mutex::scoped_lock(tail_hub_offset_lock); tail_hub_offset(1) = tail_hub_offset_y;} message() << "Tail Hub Offset y set to " << tail_hub_offset_y;}
  /// set the body z tail_hub_offset
  inline void set_tail_hub_offset_z(const double& tail_hub_offset_z) {{boost::mutex::scoped_lock(tail_hub_offset_lock); tail_hub_offset(2) = tail_hub_offset_z;} message() << "Tail Hub Offset z set to " << tail_hub_offset_z;}

  /// inertia matrix
  blas::banded_matrix<double> inertia;
  /// serialize access to inertia
  mutable boost::mutex inertia_lock;
  /// set the inertia matrix
  inline void set_inertia(const blas::banded_matrix<double>& inertia) {boost::mutex::scoped_lock(inertia_lock); this->inertia = inertia;}
  /// set the inertia in the body x
  inline void set_inertia_x(const double& jx) {{boost::mutex::scoped_lock(inertia_lock); inertia(0,0) = jx;} message() << "Inertia x set to " << jx;}
  /// set the inertia in the body y
  inline void set_inertia_y(const double& jy) {{boost::mutex::scoped_lock(inertia_lock); inertia(1,1) = jy;} message() << "Inertia y set to " << jy;}
  /// set the inertia in the body z
  inline void set_inertia_z(const double& jz) {{boost::mutex::scoped_lock(inertia_lock); inertia(2,2) = jz;} message() << "Inertia z set to " << jz;}

  ///Serializes access to the controller parameter file
  mutable boost::mutex config_file_lock;
  /// Save the configuration to the file heli::physical_param_filename
  void saveFile() const;
  /// load the parameters stored in the configuration file heli::physical_param_filename
  void loadFile();
  /// parse a vector xml node
  void parse_vector_node(rapidxml::xml_node<> *vector_node);
  /// pasrse a scalar xml node
  void parse_scalar_node(rapidxml::xml_node<> *scalar_node);

};

#endif // HELICOPTER_H
