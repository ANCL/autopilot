/**************************************************************************
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
 *************************************************************************/

#include "GPS_Filter.h"

GPS_Filter::GPS_Filter()
{
	// State and input coefficients for 2nd order Butterworth filter with 3Hz cutoff frequency (using Matlab butter fcn)
	input_coeff[0] = 0.391335772501769;
	input_coeff[1] = 0.782671545003537;
	input_coeff[2] = 0.391335772501769;
	state_coeff[0] = 0.369527377351241;
	state_coeff[1] = 0.195815712655833;
}

GPS_Filter::~GPS_Filter() {

}
