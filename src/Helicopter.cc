/**************************************************************************
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
 *************************************************************************/

#include "Helicopter.h"

Helicopter::Helicopter()
	:radio_cal_data(RadioCalibration::getInstance()),
	 out(servo_switch::getInstance()),
	 mass(13.65),
	 gravity(9.8),
	 main_hub_offset(3),
	 tail_hub_offset(3),
	 inertia(3,3)

{
	main_hub_offset.clear();
	main_hub_offset(2) = -0.32;

	tail_hub_offset.clear();
	tail_hub_offset(0) = -1.06;

	inertia(0,0) = 0.36;
	inertia(1,1) = 1.48;
	inertia(2,2) = 1.21;
}

Helicopter* Helicopter::_instance = NULL;

Helicopter* Helicopter::getInstance()
{
  if(!_instance)
      _instance = new Helicopter;

  return _instance;
}

const std::string Helicopter::PARAM_MASS = "Mass";

const std::string Helicopter::PARAM_MAIN_OFFSET_X = "Main_X";
const std::string Helicopter::PARAM_MAIN_OFFSET_Y = "Main_Y";
const std::string Helicopter::PARAM_MAIN_OFFSET_Z = "Main_Z";

const std::string Helicopter::PARAM_TAIL_OFFSET_X = "Tail_X";
const std::string Helicopter::PARAM_TAIL_OFFSET_Y = "Tail_Y";
const std::string Helicopter::PARAM_TAIL_OFFSET_Z = "Tail_Z";

const std::string Helicopter::PARAM_INERTIA_X = "J_X";
const std::string Helicopter::PARAM_INERTIA_Y = "J_Y";
const std::string Helicopter::PARAM_INERTIA_Z = "J_Z";

std::vector<Parameter> Helicopter::getParameters()
{
	std::vector<Parameter> plist;

	plist.push_back(Parameter(PARAM_MASS, get_mass(), heli::HELICOPTER_ID));

	plist.push_back(Parameter(PARAM_MAIN_OFFSET_X, get_main_hub_offset()(0), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_MAIN_OFFSET_Y, get_main_hub_offset()(1), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_MAIN_OFFSET_Z, get_main_hub_offset()(2), heli::HELICOPTER_ID));

	plist.push_back(Parameter(PARAM_TAIL_OFFSET_X, get_tail_hub_offset()(0), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_TAIL_OFFSET_Y, get_tail_hub_offset()(1), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_TAIL_OFFSET_Z, get_tail_hub_offset()(2), heli::HELICOPTER_ID));

	plist.push_back(Parameter(PARAM_INERTIA_X, get_inertia()(0,0), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_INERTIA_Y, get_inertia()(1,1), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_INERTIA_Z, get_inertia()(2,2), heli::HELICOPTER_ID));

	return plist;
}

void Helicopter::setParameter(Parameter p)
{
	std::string param_id(p.getParamID());
	boost::trim(param_id);

	if (param_id == PARAM_MASS)
		set_mass(p.getValue());
}

uint16_t Helicopter::norm2pulse(double norm, boost::array<uint16_t, 3> setpoint)
{
        uint16_t pulse = 0;
        if(setpoint[2] > setpoint[0])
          {
            if(norm >= 0)
                pulse = setpoint[1] + (norm * (setpoint[2] - setpoint[1]));
            else
                pulse = setpoint[1] - (norm * (setpoint[0] - setpoint[1]));
          }
        else if(setpoint[2] < setpoint[0])
          {
            if(norm >= 0)
                pulse = setpoint[1] - (norm * (setpoint[1] - setpoint[2]));
            else
                pulse = setpoint[1] + (norm * (setpoint[1] - setpoint[0]));
          }
        return pulse;
}

uint16_t Helicopter::norm2pulse(double norm, boost::array<uint16_t, 2> setpoint)
{
  uint16_t pulse = 0;
    {
      if(norm == 0)
        pulse = setpoint[0];
      else
        pulse = setpoint[1];
    }
  return pulse;
}

uint16_t Helicopter::norm2pulse(double norm, boost::array<uint16_t, 5> setpoint)
{
  uint16_t pulse = 1.5;
  if(setpoint[4] > setpoint[0])
    {
      if(norm <= 0.25 && norm >= 0)
          pulse = (norm/0.25) * (setpoint[1] - setpoint[0]) + setpoint[0];
      else if(norm <= 0.50 && norm > 0.25)
          pulse = ((norm - 0.25)/0.25) * (setpoint[2] - setpoint[1]) + setpoint[1];
      else if(norm <= 0.75 && norm > 0.50)
          pulse = ((norm - 0.50)/0.25) * (setpoint[3] - setpoint[2]) + setpoint[2];
      else if(norm <= 1.0 && norm > 0.75)
          pulse = ((norm - 0.75)/0.25) * (setpoint[4] - setpoint[3]) + setpoint[3];
    }
  else if(setpoint[4] < setpoint[0])
    {
    if(norm <= 0.25 && norm >= 0)
        pulse = setpoint[0] - (norm/0.25) * (setpoint[0] - setpoint[1]);
    else if(norm <= 0.50 && norm > 0.25)
        pulse = setpoint[1] - ((norm - 0.25)/0.25) * (setpoint[1] - setpoint[2]);
    else if(norm <= 0.75 && norm > 0.50)
        pulse = setpoint[2] - ((norm - 0.50)/0.25) * (setpoint[2] - setpoint[3]);
    else if(norm <= 1.0 && norm > 0.75)
        pulse = setpoint[3] - ((norm - 0.75)/0.25) * (setpoint[3] - setpoint[4]);
    }
  return pulse;
}

boost::array<uint16_t, 6> Helicopter::setScaled(boost::array<double, 6> norm)
{
  boost::array<uint16_t, 6> pulse;

  pulse[AILERON] = setAileron(norm[0]);
  pulse[ELEVATOR] = setElevator(norm[1]);
  pulse[THROTTLE] = setThrottle(norm[2]);
  pulse[RUDDER] = setRudder(norm[3]);
  pulse[GYRO] = setGyro(norm[4]);
  pulse[PITCH] = setPitch(norm[5]);

  return pulse;
}

std::vector<uint16_t> Helicopter::setScaled(std::vector<double> norm)
{
  std::vector<uint16_t> pulse(6);

  pulse[AILERON] = setAileron(norm[0]);
  pulse[ELEVATOR] = setElevator(norm[1]);
  pulse[THROTTLE] = setThrottle(norm[2]);
  pulse[RUDDER] = setRudder(norm[3]);
  pulse[GYRO] = setGyro(norm[4]);
  pulse[PITCH] = setPitch(norm[5]);

  return pulse;
}

uint16_t Helicopter::setAileron(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getAileron());
	out->setRaw(heli::CH1, pulse);
	return pulse;
}

uint16_t Helicopter::setElevator(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getElevator());
	out->setRaw(heli::CH2, pulse);
	return pulse;
}

uint16_t Helicopter::setThrottle(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getThrottle());
	out->setRaw(heli::CH3, pulse);
	return pulse;
}

uint16_t Helicopter::setRudder(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getRudder());
	out->setRaw(heli::CH4, pulse);
	return pulse;
}

uint16_t Helicopter::setGyro(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getGyro());
	out->setRaw(heli::CH5, pulse);
	return pulse;
}

uint16_t Helicopter::setPitch(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getPitch());
	out->setRaw(heli::CH6, pulse);
	return pulse;
}
