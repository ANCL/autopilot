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

/**
   \author Bryan Godbolt <godbolt@ece.ualberta.ca>
   @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
   @author Aakash Vasudevan <avasudev@ualberta.ca>
   \mainpage
   This project contains the code for the autopilot system.  This documentation is available on the
   local network at http://doc/autopilot/html/index.html .
   The central version should be used (as opposed to a locally generated version) since it is regenerated
   every time a push is made to the central repository.

   \p The contents of the root folder are
   - \b doc/ project documentation
   - \b README describes folder structure
   - \b src/ source code
   - \b utils/ sample programs to demonstrate various aspects of the system (e.g., counter board i/o)
   - \b extern/ external libraries used to build the autopilot

   @note all debugging messages should be printed using the ::debug() function.  See the Debug class.
 */


/* Boost Headers */
#include <boost/thread.hpp>

/* Project Headers */
#include "MainApp.h"

int main()
{
	MainApp m;
	boost::thread main_app(m);
	main_app.join();
	return 0;
}
