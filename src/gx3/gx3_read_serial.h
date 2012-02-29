/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
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

#ifndef GX3_READ_SERIAL_H_
#define GX3_READ_SERIAL_H_

#include "IMU.h"
/**
 * Read and interpret messages arriving on the serial port connected to the 3dm-gx3
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date February 2, 2012: Class creation
 */
class IMU::read_serial
{
public:
	void operator()();
};
#endif
