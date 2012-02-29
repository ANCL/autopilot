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

#include "Filter.h"

Filter::Filter()
:state(2, 0),
 input(2, 0)
{
	input_coeff.assign(0);
	state_coeff.assign(0);
}

double Filter::operator()(double current_input)
{
		// State estimation using 2nd order Butterworth filter (see Matlab butter fcn)
		double current_state = input_coeff[0] * current_input + input_coeff[1] * input[1] + input_coeff[2] * input[0]
					 - state_coeff[0] * state[1] - state_coeff[1] * state[0];

		state.push_back(current_state);
		input.push_back(current_input);

		return current_state;
}

void Filter::reset()
{
	state.assign(state.size(), 0);
	input.assign(input.size(), 0);
}

Filter::~Filter() {

}
