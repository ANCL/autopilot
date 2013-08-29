/**************************************************************************
 * Copyright 2013 Bryan Godbolt
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

#ifndef ADC_H_
#define ADC_H_

/* Project Headers */

/* Boost Headers */
#include <boost/thread.hpp>

/* GCC Headers */
#include <stdint.h>

/**
 * @brief Contains hardware specific code for ConnectTech ADC used for measuring battery voltage
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 *
 * This class runs one thread for polling the ADC and then has some thread safe accessors.
 * @date August 29, 2013: Class creation
 */

class ADC {
public:
	ADC();
	virtual ~ADC();
	static ADC* getInstance();

	struct update{void operator()();};

private:
	static ADC* _instance;
	static boost::mutex _instance_lock;

	/// Thread to poll adc card
	boost::thread update_thread;

	/** Store the current ampro battery voltage */
	double ampro;

	/** Store the current voltage of the receiver battery*/
	double rx;

	/** Store the current voltage of the Kontron battery */
	double kontron;

	// handle to the pci system
	int pci_handle;

	// pointer to base address of adc card
	char* adc_bar0;
};

#endif
