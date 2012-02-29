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

#include "LogFile.h"

LogFile::LogFile()
	:data_out(LogFileWrite(this)),
	 startTime(boost::posix_time::microsec_clock::local_time())
{
  log = new std::map<std::string, std::queue<std::string> >();
  headers = new std::map<std::string, std::string>();

  boost::gregorian::date d(boost::gregorian::day_clock::local_day());
  boost::gregorian::date_facet *dfacet(new boost::gregorian::date_facet("%A %B %d, %Y"));

  std::stringstream ss;
  ss.imbue(std::locale(ss.getloc(), dfacet));
  ss << d;
//  debug() << __FILE__ << __LINE__ << "Date: " << ss.str();
  std::string date_folder(ss.str());
  ss.str("");

  boost::posix_time::time_facet *tfacet(new boost::posix_time::time_facet("%H.%M.%S"));
  ss.imbue(std::locale(ss.getloc(), tfacet));
  ss << startTime;
  std::string time_folder(ss.str());
//  debug() << __FILE__ << __LINE__ << "time: " << time_folder;

  log_folder = date_folder;
  if (!boost::filesystem::exists(log_folder))
  {
	  try
	  {
		  boost::filesystem::create_directory(log_folder);
	  }
	  catch (boost::filesystem::filesystem_error& fserr)
	  {
		  critical() << "LogFile: Could not create directory: " << date_folder;
	  }
  }

  log_folder /= time_folder;

  if (!boost::filesystem::exists(log_folder))
  {
	  try
	  {
		  boost::filesystem::create_directory(log_folder);
	  }
	  catch (boost::filesystem::filesystem_error& fserr)
	  {
		  critical() << "LogFile: Could not create directory: " << date_folder;
	  }
  }
  else
  {
	  // remove the contents of the directory
	  for (boost::filesystem::directory_iterator it(log_folder); it != boost::filesystem::directory_iterator(); it++)
		  boost::filesystem::remove(*it);
  }

  MainApp::add_thread(&data_out, std::string("Log File"));
}

LogFile::~LogFile()
{
  delete log;
  delete headers;
}

LogFile* LogFile::_instance = NULL;
boost::mutex LogFile::_instance_lock;

LogFile* LogFile::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);
	if (!_instance)
		_instance = new LogFile;
	return _instance;
}

void LogFile::logHeader(const std::string& name, const std::string& header)
{
  (*headers)[name] = header;
}

boost::recursive_mutex LogFile::LogFileWrite::terminate_mutex;
bool LogFile::LogFileWrite::terminate = false;



LogFile::LogFileWrite::LogFileWrite(LogFile* parent)
	:LogFile_Pulse_Code(_PULSE_CODE_MINAVAIL + 2)
{
	if (parent == NULL)
		throw(bad_logfile_parent());
	this->parent = parent;

	MainApp::terminate.connect(LogFile::LogFileWrite::do_terminate());
}

void LogFile::LogFileWrite::write_thread()
{
		int chid = ChannelCreate(0);
		sigevent event;
		event.sigev_notify = SIGEV_PULSE;
		event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
		event.sigev_priority = getprio(0);
		event.sigev_code = LogFile_Pulse_Code;
		timer_t timer_id;
		timer_create(CLOCK_REALTIME, &event, &timer_id);
		itimerspec itime;
		itime.it_value.tv_sec = 1;
		itime.it_value.tv_nsec = 500000000;
		itime.it_interval.tv_sec = 2;
		itime.it_interval.tv_nsec = 0;
		timer_settime(timer_id, 0, &itime, NULL);

		_pulse pulse;
	while(check_terminate())
	{
		int rcvid = MsgReceivePulse(chid, &pulse, sizeof(pulse), NULL);
		if ((rcvid==0) && (pulse.code == LogFile_Pulse_Code))
		{
			std::map<std::string, std::queue<std::string> > *old_log;
			std::map<std::string, std::string> *old_headers;
			{
				boost::mutex::scoped_lock lock(parent->logMutex);
				LogFile *l = LogFile::getInstance();
				old_log = l->log;
				old_headers = l->headers;
				l->log = new std::map<std::string, std::queue<std::string> >();
				l->headers = new std::map<std::string, std::string>();
			} // let lock go out of scope (to unlock mutex)
			write(old_log, old_headers);
			delete old_log;
			delete old_headers;
		}
		boost::thread::yield();
	}
	// close open files
	for (std::map<std::string, std::fstream*>::iterator it = openFiles.begin(); it != openFiles.end(); ++it)
	{
		debug() << "LogFileWrite: Closing " << it->first;
		it->second->close();
	}
}

bool LogFile::LogFileWrite::check_terminate()
{
	bool t = false;
	terminate_mutex.lock();
	t = terminate;
	terminate_mutex.unlock();
	return !t;
}


void LogFile::LogFileWrite::write(std::map<std::string, std::queue<std::string> > * log, std::map<std::string, std::string> * headers)
{
	for (std::map<std::string, std::queue<std::string> >::iterator it = log->begin();
			it != log->end();
			++it)
	{
		if (openFiles.count(it->first) == 0)
		{
			openFiles[it->first] = new std::fstream;
			boost::filesystem::path filename;
			if (!(boost::filesystem::path(it->first)).has_extension())//?it->first : it->first + ".dat");
				filename = LogFile::getInstance()->log_folder / (it->first + ".dat");
			else
				filename = LogFile::getInstance()->log_folder / it->first;

			try
			{
				if (!boost::filesystem::exists(filename))
				{
					debug() << "LogFileWrite: Creating log file " << filename.c_str();
					openFiles[it->first]->open(filename.c_str(), std::fstream::out);
					(*openFiles[it->first]) << "Time(ms)\t";
					if (headers->count(it->first) > 0)
						(*openFiles[it->first]) << (*headers)[it->first];
					(*openFiles[it->first]) << std::endl;
				}
				else
					openFiles[it->first]->open(filename.c_str(), std::fstream::out | std::fstream::ate | std::fstream::app);
			}
			catch (boost::filesystem::filesystem_error& fserr)
			{
				critical() << fserr.what();
			}
		}

		while(!it->second.empty())
		{
			(*openFiles[it->first]) << it->second.front();
			it->second.pop();
		}
	}
}

void LogFile::LogFileWrite::do_terminate::operator()()
{
	  	terminate_mutex.lock();
	  	terminate = true;
	  	message() << "LogFileWrite: Received terminate signal.  Closing log files.";
	  	terminate_mutex.unlock();
}

void LogFile::logMessage(const std::string& name, const std::string& msg)
{
  std::stringstream *dataStr = new std::stringstream();

  boost::posix_time::ptime time(boost::posix_time::microsec_clock::local_time());
  *dataStr << ((time-startTime).total_milliseconds()) << '\t';
  *dataStr << msg;
  *dataStr << std::endl;
  {
	  boost::mutex::scoped_lock(this->logMutex);
  	  (*log)[name].push(dataStr->str());
  }
  delete dataStr;
}
