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

#ifndef MAINAPP_H_
#define MAINAPP_H_

/* STL Headers */
#include <iostream> // used only while quick debugging to print to screen.
#include <string>
#include <vector>

/* Headers needed for timer */
#include <stdio.h>      // used for quick debugging only.
#include <time.h>
#include <signal.h>
#include <sys/netmgr.h>
#include <sys/neutrino.h>
#include <sched.h>      // for setting thread priorities.

/* Project Headers */
#include "heli.h"

/* Boost Headers */
#include <boost/foreach.hpp>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

/**
 * \brief This class replaces the standard main function and implements the main program logic.
 * \author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * \date June 15, 2011 Class created
 * @date January 20, 2012 Added pilot mode switch from new takeover
 *
 * This class is necessary in order to facilitate thread cleanup.  In particular, when this object
 * is destroyed (because the main function is about to return) boost::this_thread::at_thread_exit()
 * is used to call a cleanup routine which sends a terminate signal to other threads in the program
 * then waits for them to terminate.  The signal is sent using the (thread safe) boost::signals2 library.
 * In order for the cleanup class to wait on a thread, the thread must have identified itself at some point
 * by calling the MainApp::add_thread function and passing a pointer to itself and its name (the latter is
 * used for printing messages to the user).
 */

class MainApp {
public:

	/// default constructor (initializes terminate to false)
	MainApp();

	/// thread safe copy constructor
	MainApp(const MainApp& other);

	/// thread safe assignment operator
	const MainApp& operator=(const MainApp& other);

	/// functor to create thread
	void operator()(){run();}

	/// Append a thread to a list so that MainApp can wait for it after sending terminate signal
	static void add_thread(boost::thread *thread, std::string name);
	/// Terminate signal used to tell other threads the program is about to terminate
	static boost::signals2::signal<void ()> terminate;

	/// signal send by main app to notify other threads of a mode change (in particular qgclink::qgcsend)
	static boost::signals2::signal<void (heli::AUTOPILOT_MODE)> mode_changed;

	/// signal to request a mode change from other threads
	static boost::signals2::signal<void (heli::AUTOPILOT_MODE)> request_mode;

	/// Data structure for storing a boost::thread*, thread name pair
	class ThreadName
	{
	public:
		ThreadName(boost::thread * = new boost::thread(), std::string name = std::string());
		boost::thread *thread;
		std::string name;
	};

private:
	/// Function in which to place main program logic (replaces main()).
	void run();

	/// List of other threads in the program which should be allowed to terminate before the main program exits
	static std::vector<ThreadName> threads;

	/// controls whether the main loop continues to execute
	bool _terminate;

	/// synchronizes thread access to MainApp::_terminate
	mutable boost::mutex terminate_lock;

	/// returns the value of _terminate using MainApp::terminate_lock for synchronization
	bool check_terminate();

	/// class to send terminate signal and then wait for other threads when run() finishes
	class cleanup
	{
	public:
		void operator()();
	};

	/// slot connected to MainApp::terminate which sets MainApp::_terminate to true
	class do_terminate
	{
	public:
		do_terminate(MainApp* parent=NULL) {this->parent=parent;}
		void operator()();
	private:
		MainApp *parent;
	};



	/// stores the current operating mode of the autopilot
	heli::AUTOPILOT_MODE autopilot_mode;

	/// synchronizes thread access to MainApp::autopilot_mode
	mutable boost::mutex autopilot_mode_lock;

	/// @returns the value of MainApp::autopilot_mode using MainApp::autopilot_mode_lock
	int getMode();

	/// @returns the string representation of the current mode
	std::string getModeString();

	/// @returns the string representation of mode
	static std::string getModeString(heli::AUTOPILOT_MODE mode);

	/// slot connected to MainApp::request_mode to change the value of MainApp::autopilot_mode
	class change_mode
	{
	public:
		change_mode(MainApp *parent = NULL) {this->parent = parent;}
		void operator()(heli::AUTOPILOT_MODE mode);
	private:
		MainApp *parent;
	};

	void change_pilot_mode(heli::PILOT_MODE mode);


};

#endif /* MAINAPP_H_ */
