#include "IMU.h"
#include "LogFile.h"
#include <boost/assign.hpp>
#include "adc.h"
using namespace boost::assign;
#include <mavlink.h>
void MainApp::run()
{
//	signal(SIGINT, heli::shutdown);             // Shutdown program by sending a SIGINT.
//	boost::this_thread::at_thread_exit(cleanup());

//	do_terminate terminate_slot(this);
//	terminate.connect(terminate_slot);

	debug() << "test";
	ADC adc;


//	while(check_terminate())
//	{
//		boost::this_thread::sleep(boost::posix_time::seconds(1));
//	}

}
