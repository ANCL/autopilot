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

#include "QGCSend.h"

/* Project Headers */
#include "RCTrans.h"
#include "Control.h"
#include "MainApp.h"
#include "IMU.h"
#include "GPS.h"

/* MAVLink Headers */
#include <mavlink.h>

/* STL Headers */
#include <vector>

/* Boost Headers */
#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/algorithm/string.hpp>

QGCLink::QGCSend::QGCSend(QGCLink* parent)
:qgc(parent),
 servo_source(heli::NUM_AUTOPILOT_MODES),
 pilot_mode(heli::NUL_PILOT_MODES),
 filter_state(IMU::NUM_GX3_MODES),
 control_mode(heli::Num_Controller_Modes),
 attitude_source(true),
 startTime(boost::posix_time::microsec_clock::local_time())
{
	send_queue = new std::queue<std::vector<uint8_t> >();
}
QGCLink::QGCSend::QGCSend(const QGCSend& other)
:qgc(other.qgc),
 startTime(other.startTime)
{
	{
		boost::mutex::scoped_lock scoped_lock(other.servo_source_lock);
		servo_source = other.servo_source;
	}
	{
		boost::mutex::scoped_lock lock(other.pilot_mode_lock);
		pilot_mode = other.pilot_mode;
	}
	{
		boost::mutex::scoped_lock lock(other.filter_state_lock);
		filter_state = other.filter_state;
	}
	{
		boost::mutex::scoped_lock lock(other.control_mode_lock);
		control_mode = other.control_mode;
	}
	{
		boost::mutex::scoped_lock lock(other.attitude_source_lock);
		attitude_source = other.attitude_source;
	}

	send_queue = new std::queue<std::vector<uint8_t> >(*other.send_queue);
}
QGCLink::QGCSend::~QGCSend()
{
	delete send_queue;
}

QGCLink::QGCSend& QGCLink::QGCSend::operator=(const QGCLink::QGCSend& other)
{
	if (this == &other)
		return *this;

	{
		boost::mutex::scoped_lock lock1(&servo_source_lock < &other.servo_source_lock ? servo_source_lock: other.servo_source_lock);
		boost::mutex::scoped_lock lock2(&servo_source_lock > &other.servo_source_lock ? servo_source_lock : other.servo_source_lock);
		servo_source = other.servo_source;
	}
	{
		boost::mutex::scoped_lock lock1(&pilot_mode_lock < &other.pilot_mode_lock ? pilot_mode_lock: other.pilot_mode_lock);
		boost::mutex::scoped_lock lock2(&pilot_mode_lock > &other.pilot_mode_lock ? pilot_mode_lock : other.pilot_mode_lock);
		pilot_mode = other.pilot_mode;
	}
	{
		boost::mutex::scoped_lock lock1(&filter_state_lock < &other.filter_state_lock ? filter_state_lock: other.filter_state_lock);
		boost::mutex::scoped_lock lock2(&filter_state_lock > &other.filter_state_lock ? filter_state_lock : other.filter_state_lock);
		filter_state = other.filter_state;
	}
	{
		boost::mutex::scoped_lock lock1(&control_mode_lock < &other.control_mode_lock ? control_mode_lock: other.control_mode_lock);
		boost::mutex::scoped_lock lock2(&control_mode_lock > &other.control_mode_lock ? control_mode_lock : other.control_mode_lock);
		control_mode = other.control_mode;
	}
	{
		boost::mutex::scoped_lock lock1(&attitude_source_lock < &other.attitude_source_lock ? attitude_source_lock: other.attitude_source_lock);
		boost::mutex::scoped_lock lock2(&attitude_source_lock > &other.attitude_source_lock ? attitude_source_lock : other.attitude_source_lock);
		attitude_source = other.attitude_source;
	}

	return *this;
}

void QGCLink::QGCSend::send()
{


  int chid = ChannelCreate(0);
  sigevent event;
  event.sigev_notify = SIGEV_PULSE;
  event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
  event.sigev_priority = heli::qgcSendPriority; //85;//getprio(0);
  event.sigev_code = heli::QGCLinkSendPulseCode;
  timer_t timer_id;
  timer_create(CLOCK_REALTIME, &event, &timer_id);
  itimerspec itime;

  int send_period = 5000000;
  itime.it_value.tv_sec = 0;
  itime.it_value.tv_nsec = send_period;
  itime.it_interval.tv_sec = 0;
  itime.it_interval.tv_nsec = send_period;
  timer_settime(timer_id, 0, &itime, NULL);

  int send_rate = 1/(static_cast<double>(send_period)/1000000000); // send freq in hz
  _pulse pulse;

  if (qgc == NULL)
	  qgc = QGCLink::getInstance();

  int loop_count = 0;

  // connect signals to track system mode
  boost::signals2::scoped_connection control_mode_connection(Control::getInstance()->mode_changed.connect(
		  boost::bind(&QGCLink::QGCSend::set_control_mode, this, _1)));
  boost::signals2::scoped_connection servo_source_connection(MainApp::mode_changed.connect(
		  boost::bind(&QGCLink::QGCSend::set_servo_source, this, _1)));
  boost::signals2::scoped_connection pilot_mode_connection(servo_switch::getInstance()->pilot_mode_changed.connect(
		  boost::bind(&QGCLink::QGCSend::set_pilot_mode, this, _1)));
  boost::signals2::scoped_connection gx3_state_connection(IMU::getInstance()->gx3_mode_changed.connect(
		  boost::bind(&QGCLink::QGCSend::set_filter_state, this, _1)));
  boost::signals2::scoped_connection gx3_message_connection(IMU::getInstance()->gx3_status_message.connect(
		  boost::bind(&QGCLink::QGCSend::set_gx3_message, this, _1)));
  attitude_source_connection = QGCLink::getInstance()->attitude_source.connect(
		  boost::bind(&QGCLink::QGCSend::set_attitude_source, this, _1));
  boost::signals2::scoped_connection warning_connection(Debug::warning.connect(
		  boost::bind(&QGCLink::QGCSend::message_queue_push, this, _1)));
  boost::signals2::scoped_connection critical_connection(Debug::critical.connect(
		  boost::bind(&QGCLink::QGCSend::message_queue_push, this, _1)));

  for (;;)
  {
	  int rcvid = MsgReceivePulse(chid, &pulse, sizeof(pulse), NULL);
	  if ((rcvid == 0) && (pulse.code == heli::QGCLinkSendPulseCode))
	  {

		  if (should_run(qgc->get_heartbeat_rate(), send_rate, loop_count))
		  {
			  send_heartbeat(send_queue);
			  send_status(send_queue);
		  }

		  if (get_gx3_new_message())
			  send_gx3_message(send_queue);

		  if(should_run(qgc->get_attitude_rate(), send_rate, loop_count))
			  send_attitude(send_queue);

		  /* Send parameters */
		  qgc->param_recv_lock.lock();
		  bool param_recv = qgc->param_recv;
		  qgc->param_recv = false;
		  qgc->param_recv_lock.unlock();
		  if(param_recv)
			  send_param(send_queue);

		  /* Send RC Channels */
		  if (should_run(qgc->get_rc_channel_rate(), send_rate, loop_count))
			  send_rc_channels(send_queue);

		  /* send control effort */
		  if (should_run(qgc->get_control_output_rate(), send_rate, loop_count))
			  send_control_effort(send_queue);

		  if (should_run(qgc->get_position_rate(), send_rate, loop_count))
			  send_position(send_queue);

		  qgc->requested_params_lock.lock();
		  if (!qgc->requested_params.empty())
			  send_requested_params(send_queue);
		  qgc->requested_params_lock.unlock();

		  /* Send RC Calibration */
		  if (qgc->get_requested_rc_calibration())
		  {
			  send_rc_calibration(send_queue);
			  qgc->clear_requested_rc_calibration();
		  }

		  /* Send any queued messages to the console */
		  if (!message_queue_empty()) //only send max one message per iteration
			  send_console_message(message_queue_pop(), send_queue);
		  try
		  {
			  /* actually send data to qgc */
			  while (!send_queue->empty())
			  {

				  qgc->socket.send_to(boost::asio::buffer(send_queue->front()), qgc->qgc);
				  send_queue->pop();
			  }

		  }
		  catch (boost::system::system_error e)
		  {
			  warning() << e.what();
		  }
		  /* Increment loop count */
		  loop_count++;
	  }
  }
}

void QGCLink::QGCSend::send_position(std::queue<std::vector<uint8_t> > *sendq)
{
	{
		IMU* imu = IMU::getInstance();

		// get llh pos
		blas::vector<double> _llh_pos(imu->get_llh_position());
		std::vector<float> llh_pos(_llh_pos.begin(), _llh_pos.end());
		// get ned pos
		blas::vector<double> _ned_pos(imu->get_ned_position());
		std::vector<float> ned_pos(_ned_pos.begin(), _ned_pos.end());
		// get ned vel
		blas::vector<double> _ned_vel(imu->get_velocity());
		std::vector<float> ned_vel(_ned_vel.begin(), _ned_vel.end());
		// get ned origin
		blas::vector<double> _ned_origin(imu->get_llh_origin());
		std::vector<float> ned_origin(_ned_origin.begin(), _ned_origin.end());

		mavlink_message_t msg;
		std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

		mavlink_msg_ualberta_position_pack(qgc->uasId, heli::GX3_ID, &msg,
				&llh_pos[0], &ned_pos[0], &ned_vel[0], &ned_origin[0],
				(boost::posix_time::microsec_clock::local_time() - startTime).total_milliseconds());

		buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
		sendq->push(buf);
	}

	{

		GPS* gps = GPS::getInstance();

		blas::vector<double> _pos_error(gps->get_pos_sigma());
//		debug() << "sent pos error:" << _pos_error;
		std::vector<float> pos_error(_pos_error.begin(), _pos_error.end());

		blas::vector<double> _vel_error(gps->get_vel_sigma());
		std::vector<float> vel_error(_vel_error.begin(), _vel_error.end());

		mavlink_message_t msg;
		std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

		mavlink_msg_novatel_gps_raw_pack(qgc->uasId, heli::NOVATEL_ID, &msg,
								       gps->get_position_type(), gps->get_position_status(),
								       gps->get_num_sats(), &pos_error[0],
								       gps->get_velocity_type(), &vel_error[0],
								       (boost::posix_time::microsec_clock::local_time() - startTime).total_milliseconds());
		buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
		sendq->push(buf);
	}
}

void QGCLink::QGCSend::send_param(std::queue<std::vector<uint8_t> > *sendq)
{
	debug() << "attempting to send parameter list";
	mavlink_message_t msg;
	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

	std::vector<std::vector<Parameter> > plist;

	plist.push_back(Control::getInstance()->getParameters());

	int num_params = 0;
	for (unsigned int i=0; i<plist.size(); i++)
		num_params += plist[i].size();
	int index = 0;
	for (unsigned int i=0; i<plist.size(); i++)
		for (unsigned int j=0; j<plist.at(i).size(); j++)
		{
			mavlink_msg_param_value_pack(qgc->uasId, (uint8_t)plist.at(i).at(j).getCompID(), &msg, (const char*)(plist.at(i).at(j).getParamID().c_str()),
					plist.at(i).at(j).getValue(), MAV_VAR_FLOAT, num_params, index);
			buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
			sendq->push(buf);
			index++;
		}
}

void QGCLink::QGCSend::send_attitude(std::queue<std::vector<uint8_t> > *sendq)
{
	IMU* imu = IMU::getInstance();
	blas::vector<double> _nav_euler(imu->get_nav_euler());
	std::vector<float> nav_euler(_nav_euler.begin(), _nav_euler.end());

//	blas::vector<double> euler_rate(imu->get_euler_rate());
	blas::vector<double> _nav_ang_rate(imu->get_nav_angular_rate());
	std::vector<float> nav_ang_rate(_nav_ang_rate.begin(), _nav_ang_rate.end());

	blas::vector<double> _ahrs_euler(imu->get_ahrs_euler());
	std::vector<float> ahrs_euler(_ahrs_euler.begin(), _ahrs_euler.end());

	blas::vector<double> _ahrs_ang_rate(imu->get_ahrs_angular_rate());
	std::vector<float> ahrs_ang_rate(_ahrs_ang_rate.begin(), _ahrs_ang_rate.end());

	mavlink_message_t msg;
	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

	mavlink_msg_ualberta_attitude_pack(qgc->uasId, heli::GX3_ID, &msg,
			&nav_euler[0], &nav_ang_rate[0], &ahrs_euler[0], &ahrs_ang_rate[0],
			(boost::posix_time::microsec_clock::local_time() - startTime).total_milliseconds());

//	mavlink_msg_attitude_pack(qgc->uasId, heli::GX3_ID, &msg,
//			(boost::posix_time::microsec_clock::local_time() - startTime).total_milliseconds(),
//			euler[0], euler[1], euler[2], euler_rate[0], euler_rate[1], euler_rate[2]);

	buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
	sendq->push(buf);
}

void QGCLink::QGCSend::send_requested_params(std::queue<std::vector<uint8_t> > *sendq)
{
	mavlink_message_t msg;
	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

	while(!qgc->requested_params.empty())
	{
		mavlink_msg_param_value_pack(100, qgc->requested_params.front().getCompID(), &msg, (const char*)(qgc->requested_params.front().getParamID().c_str()),
				qgc->requested_params.front().getValue(), MAV_VAR_FLOAT, 1/*num_params*/, -1/*index*/);

		buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
		sendq->push(buf);
		qgc->requested_params.pop();
	}
}

void QGCLink::QGCSend::send_rc_calibration(std::queue<std::vector<uint8_t> > *sendq)
{
	mavlink_message_t msg;
	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
	RadioCalibration *radio = RadioCalibration::getInstance();

	mavlink_msg_radio_calibration_pack(qgc->uasId, heli::RADIO_CAL_ID, &msg,
			radio->getAileron().data(),
			radio->getElevator().data(),
			radio->getRudder().data(),
			radio->getGyro().data(),
			radio->getPitch().data(),
			radio->getThrottle().data()
			);

	buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
	sendq->push(buf);
}

bool QGCLink::QGCSend::should_run(int stream_rate, int send_rate, int count)
{
	if (stream_rate == 0 || stream_rate > send_rate)
		return false;
	return count%(send_rate/stream_rate) == 0;
}

void QGCLink::QGCSend::send_raw_imu(std::queue<std::vector<uint8_t> > *sendq)
{
//	NavFilter *nav = NavFilter::getInstance();
//
//	NavFilter::IMU_Data data = nav->getCurrentRawIMU();
//
//	mavlink_message_t msg;
//	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
//
//	mavlink_msg_scaled_imu_pack(100, 200, &msg, data.timerticks(),
//			data.accel()[0]*1000, data.accel()[1]*1000, data.accel()[2]*1000,
//			data.gyro()[0]*1000, data.gyro()[1]*1000, data.gyro()[2]*1000,
//			data.mag()[0]/10, data.mag()[1]/10, data.mag()[2]/10);
//
//	buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
//
//
//	sendq->push(buf);

}

void QGCLink::QGCSend::send_heartbeat(std::queue<std::vector<uint8_t> > *sendq)
{
	int system_type = MAV_TYPE_HELICOPTER;
	int autopilot_type = MAV_AUTOPILOT_UALBERTA;

	mavlink_message_t msg;
	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type, 0, 0, 0);
	buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));

	sendq->push(buf);

}

void QGCLink::QGCSend::send_rc_channels(std::queue<std::vector<uint8_t> > *sendq)
{
	{
		std::vector<uint16_t> raw(servo_switch::getInstance()->getRaw());
		mavlink_message_t msg;
		std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

		mavlink_msg_rc_channels_raw_pack(100, 200, &msg,
				0, 0,
				raw[0], raw[1], raw[2], raw[3],
				raw[4], raw[5], raw[6], raw[7], 0);
		buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
		sendq->push(buf);
	}
	{
		boost::array<double, 6> scaled(RCTrans::getScaledArray());

		mavlink_message_t msg;
		std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

		mavlink_msg_rc_channels_scaled_pack(100, 200, &msg,
				0,0,
				static_cast<int16_t>(scaled[RCTrans::AILERON]*1e4),
				static_cast<int16_t>(scaled[RCTrans::ELEVATOR]*1e4),
				static_cast<int16_t>(scaled[RCTrans::THROTTLE]*1e4),
				static_cast<int16_t>(scaled[RCTrans::RUDDER]*1e4),
				static_cast<int16_t>(scaled[RCTrans::GYRO]*1e4),
				static_cast<int16_t>(scaled[RCTrans::PITCH]*1e4),
				0,0,0);
		buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
		sendq->push(buf);
	}
}

void QGCLink::QGCSend::send_status(std::queue<std::vector<uint8_t> >* sendq)
{

	// get elements of the system status
	heli::AUTOPILOT_MODE servo_source = get_servo_source();
	uint8_t qgc_servo_source = 255;
	switch(servo_source)
	{
	case heli::MODE_DIRECT_MANUAL:
		qgc_servo_source = ::UALBERTA_MODE_MANUAL_DIRECT;
		break;
	case heli::MODE_SCALED_MANUAL:
		qgc_servo_source = ::UALBERTA_MODE_MANUAL_SCALED;
		break;
	case heli::MODE_AUTOMATIC_CONTROL:
		qgc_servo_source = ::UALBERTA_MODE_AUTOMATIC_CONTROL;
		break;
	default:
		break;
	}

	heli::PILOT_MODE pilot_mode = get_pilot_mode();
	uint8_t qgc_pilot_mode = 255;
	switch(pilot_mode)
	{
	case heli::PILOT_MANUAL:
		qgc_pilot_mode = ::UALBERTA_PILOT_MANUAL;
		break;
	case heli::PILOT_AUTO:
		qgc_pilot_mode = ::UALBERTA_PILOT_AUTO;
		break;
	default:
		break;
	}

	IMU::GX3_MODE filter_state = get_filter_state();
	uint8_t qgc_filter_state = 255;
	switch(filter_state)
	{
	case IMU::STARTUP:
		qgc_filter_state = ::UALBERTA_GX3_STARTUP;
		break;
	case IMU::INIT:
		qgc_filter_state = ::UALBERTA_GX3_INIT;
		break;
	case IMU::RUNNING:
		qgc_filter_state = ::UALBERTA_GX3_RUNNING_VALID;
		break;
	case IMU::ERROR:
		qgc_filter_state = ::UALBERTA_GX3_RUNNING_ERROR;
		break;
	default:
		break;
	}

	heli::Controller_Mode control_mode = get_control_mode();
	uint8_t qgc_control_mode = 255;
	switch(control_mode)
	{
	case heli::Mode_Attitude_Stabilization_PID:
		qgc_control_mode = ::UALBERTA_ATTITUDE_PID;
		break;
	case heli::Mode_Position_Hold_PID:
		qgc_control_mode = ::UALBERTA_TRANSLATION_PID;
		break;
	default:
		break;
	}

	mavlink_message_t msg;
	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

	mavlink_msg_ualberta_sys_status_pack(qgc->uasId, 200, &msg,
			qgc_servo_source, qgc_filter_state, qgc_pilot_mode, qgc_control_mode,(get_attitude_source()?UALBERTA_NAV_FILTER:UALBERTA_AHRS),
			servo_switch::getInstance()->get_engine_rpm(), servo_switch::getInstance()->get_main_rotor_rpm(), 0, 0);

	buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));

	sendq->push(buf);
}

void QGCLink::QGCSend::send_control_effort(std::queue<std::vector<uint8_t> > *sendq)
{
	mavlink_message_t msg;
	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

	blas::vector<double> effort(Control::getInstance()->get_control_effort());
	std::vector<float> control(effort.begin(), effort.end());

	mavlink_msg_ualberta_control_effort_pack(qgc->uasId, heli::CONTROLLER_ID, &msg, &control[0]);




	buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
	sendq->push(buf);
}

void QGCLink::QGCSend::send_gx3_message(std::queue<std::vector<uint8_t> > *sendq)
{
	std::string message(get_gx3_message());
	clear_gx3_new_message();
	message.resize(49); // leave room for \0
	mavlink_message_t msg;
	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

	::mavlink_msg_ualberta_gx3_message_pack(qgc->uasId, heli::GX3_ID, &msg, message.c_str());
	buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
	// send twice
	sendq->push(buf);
	sendq->push(buf);
}

std::string QGCLink::QGCSend::message_queue_pop()
{
	boost::mutex::scoped_lock lock(message_queue_lock);
	std::string message(message_queue.front());
	message_queue.pop();
	return message;
}

void QGCLink::QGCSend::send_console_message(const std::string& message, std::queue<std::vector<uint8_t> > *sendq)
{
	std::string console(message);
	console.resize(50);

	mavlink_message_t msg;
	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

	::mavlink_msg_statustext_pack(qgc->uasId, 0, &msg, (boost::algorithm::starts_with(console, "Critical")?255:0), console.c_str());
	buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
	sendq->push(buf);
}
