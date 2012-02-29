/*******************************************************************************
 * Copyright 2012 Bryan Godbolt, Nikos Vitzilaios
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

#ifndef FILTER_H_
#define FILTER_H_

/* Boost Headers*/
#include <boost/circular_buffer.hpp>
#include <boost/array.hpp>

/**
 * @brief Data filtering
 *
 * This class implements data filtering for IMU and GPS data.
 * It is designed to use second order filters so it stores appropriate previous input and state data.
 * After you set filter coefficients to an object of this class (IMU_filter, GPS_Filter),
 * it calculates filtered data as an output to any current input and state data.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
 * @date January 18, 2012
 */

class Filter {
public:
	/**
	 * @param cutoff frequency in Hertz.
	 */
	Filter();
	virtual ~Filter() = 0;

	double operator()(double current_input);

	void reset();
protected:
	boost::circular_buffer<double> state;
	boost::circular_buffer<double> input;
	boost::array<double, 3> input_coeff;
	boost::array<double, 2> state_coeff;

};

#endif
