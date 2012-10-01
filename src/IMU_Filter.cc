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

#include "IMU_Filter.h"

#include <algorithm>

IMU_Filter::IMU_Filter()
: inputs(64, 0)
{
	inputs.assign(inputs.size(), 0);
	numerator_coeffs.assign(0);

	const double filter_coeffs[64] = {
	  -3.178415947123e-05,0.0001224319755866,0.0002004404680017,0.0003488471460631,
	  0.0005671834873015,0.0008686374904609, 0.001269751997279, 0.001787934729854,
	    0.00244046120474, 0.003243709269341, 0.004212343496969, 0.005358457175968,
	   0.006690726110351, 0.008213604807752,  0.00992659815178,  0.01182366416853,
	    0.01389279947716,  0.01611582695183,   0.0184683888342,   0.0209201673573,
	    0.02343537117009,  0.02597349777071,  0.02849032796682,  0.03093908790592,
	    0.03327174706043,   0.0354404515294,  0.03739905943818,  0.03910467033843,
	    0.04051901737011,  0.04160967792596,  0.04235122582904,  0.04272655856899,
	    0.04272655856899,  0.04235122582904,  0.04160967792596,  0.04051901737011,
	    0.03910467033843,  0.03739905943818,   0.0354404515294,  0.03327174706043,
	    0.03093908790592,  0.02849032796682,  0.02597349777071,  0.02343537117009,
	     0.0209201673573,   0.0184683888342,  0.01611582695183,  0.01389279947716,
	    0.01182366416853,  0.00992659815178, 0.008213604807752, 0.006690726110351,
	   0.005358457175968, 0.004212343496969, 0.003243709269341,  0.00244046120474,
	   0.001787934729854, 0.001269751997279,0.0008686374904609,0.0005671834873015,
	  0.0003488471460631,0.0002004404680017,0.0001224319755866,-3.178415947123e-05
	};

	std::copy(filter_coeffs, filter_coeffs + 64, numerator_coeffs.begin());
}

double IMU_Filter::operator()(double current_input)
{
	inputs.push_back(current_input);

	double output = 0;
	for (int i = 0; i< 64; ++i)
		output += numerator_coeffs[i] * inputs[63-i];
	return output;
}

void IMU_Filter::reset()
{
	inputs.assign(inputs.size(), 0);
}

IMU_Filter::~IMU_Filter()
{

}
