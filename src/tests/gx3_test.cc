#include "IMU.h"
#include "LogFile.h"
#include <boost/assign.hpp>
using namespace boost::assign;
#include <mavlink.h>
void MainApp::run()
{
	signal(SIGINT, heli::shutdown);             // Shutdown program by sending a SIGINT.
	boost::this_thread::at_thread_exit(cleanup());

	do_terminate terminate_slot(this);
	terminate.connect(terminate_slot);


	QGCLink::getInstance();
	LogFile::getInstance();
	IMU::getInstance();
	servo_switch::getInstance();

//	std::vector<uint8_t> buf;
//	buf += 63, 24, 254, 4, 5, 100, 200, 222, 255, 255, 255, 255, 201, 118, 78, 90;
//	mavlink_message_t message;
//	mavlink_status_t status;
//	for (int i=0; i<buf.size(); i++)
//	{
//		int decoded = mavlink_parse_char(MAVLINK_COMM_0, buf[i], &message, &status);
//		if (decoded)
//		{
//			debug() << "decoded message id: " << (int)message.msgid;
//		}
//	}

	while(check_terminate())
	{
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}

}
