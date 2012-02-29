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

#ifndef LOGFILE_H
#define LOGFILE_H

/* STL Headers */
#include <queue>
#include <vector>
#include <sstream>
#include <map>
#include <iostream>
#include <fstream>
#include <exception>

/* boost headers */
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/signals2.hpp>
#include <boost/filesystem.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

/* c headers */
#include <stdint.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <time.h>

/* Project headers */
#include "MainApp.h"
#include "heli.h"
#include "Debug.h"

/**
   \brief This class implements the logging facility for the avionics.

   \author Bryan Godbolt <godbolt@ece.ualberta.ca>
   \date March 1, 2011: Creation of class
   \date June 18, 2011: Add separate writing thread
   @date October 20, 2011: Add boost::array log function

   This class uses the singleton design pattern to enforce the user to only
   instantiate one object.  In order to get the instance of the class you can
   declare a variable as follows,
   \code
   LogFile *log = LogFile::getInstance();
   \endcode
   Once you have a pointer to the instance of the LogFile you can just add
   vectors to a log of any type.  There is no further initialization necessary.
   \code
   vector<float> data_vector;
   // put data in data_vector for current iteration
   log->logData("file name", data_vector);
   \endcode
   The string passed as filename internally identifies a queue where the vector
   is stored.  Every time you call LogFile::logData() the data vector is appended
   to the queue identified by the file name string.

   \warning The data type of the vector passed to LogFile::logData() must implement
   the \code ostream& operator<<(ostream&, T&) \endcode
   operator, where T refers the type of the data vector.
   \note This operator is defined for all basic types (e.g., int, float, etc)

   Each file can optionally be assigned a header string using LogFile::logHeader()
   which will be written out at the top of the log file.  LogFile::logHeader() must be
   called before the first call to LogFile::logData for a particular file.

	The data write is performed in a seperate thread.  This means when the program is going to terminate
	the LogFile object (or more precisely the LogFileWrite object) must be allow to
	finish writing any data it has in its buffer which has not yet been written.
	Currently this functionality is implemented by MainApp::terminate which sends a signal
	which is received by LogFileWrite when the program is about to exit, and by MainApp::add_thread()
	which allows LogFileWrite to tell MainApp it exists so that the main program can wait
	until LogFileWrite terminates before exiting.

   \code
	using std::vector;

	vector<float> norms(8);

	LogFile *log = LogFile::getInstance();

	boost::this_thread::at_thread_exit(cleanup());

	log->logHeader("normalized outputs", "CH1(us)\tCH2(us)\tCH3(us)\tCH4(us)\tCH5(us)\tCH6(us)\tCH7(us)\tCH8(us)");
	for (int i=0; i<10; i++)
	{
		BOOST_FOREACH(float & norm, norms)
		{
			norm = rand() % 1000 + 1000;
		}
		log->logData("normalized outputs", norms);
		boost::this_thread::sleep(boost::posix_time::seconds(3));
	}
   \endcode

   \todo Add the ability to set a base directory
   \todo Create a subdirectory using a date/time stamp to store the output data
 */

class LogFile
{

 public:
  /// Destructor: frees memory used by the internal data structures
  ~LogFile();

  /** Returns the instance of the LogFile object using the singleton
      design pattern
  */
  static LogFile* getInstance();

  /**
     Assigns a header to a log file
     \param name log file to assign a name to
     \param header string to be written at the head of the log file
     This function must be called before the first call to logData in order to have an effect.
  */
  void logHeader(const std::string& name, const std::string& header);

//  /**
//     Appends data in a vector to a log file
//     \param name log file to append data to
//     \param data vector of data to append
//  */
//  template<typename T>
//    void logData(const std::string& name, const std::vector<T>& data);
//  /**
//   * Appends data in a boost::array to a log file
//   * @note the same log file can be appended with either
//   * std::vector or boost::aray data
//   * @param name name of log file to append data to
//   * @param data with data to append
//   */
//  template<typename T, std::size_t N>
//  void logData(const std::string& name, const boost::array<T,N>& data);

  /**
   * Template log function which logs any data container that supports
   * const iterators
   */
  template<typename DataContainer>
  void logData(const std::string& name, const DataContainer& data);
  /**
   * Allows message logging by appending msg to the log, name
   * @param name log file to append message to
   * @param msg message to append to name
   */
  void logMessage(const std::string& name, const std::string& msg);
 protected:

  /// stores the log data
  std::map<std::string, std::queue<std::string> > *log;
  /// stores the headers for the log files
  std::map<std::string, std::string> *headers;

 private:
  /// Singleton constructor: allocates memory for internal data structures
  LogFile();
  /// stores a pointer to the instance of this class
  static LogFile* _instance;
  /// Mutex to make class instantiation threadsafe
  static boost::mutex _instance_lock;

  /// Object to manage the thread that does the actual writing to disk
  boost::thread data_out;
  /// Stores the time when the class is instantiated (i.e., the program starts)
  boost::posix_time::ptime startTime;
  /// Stores the folder name to store the log files in
  boost::filesystem::path log_folder;
  /// Mutex to synchronize the writing thread with the logging (main or other) thread
  boost::mutex logMutex;


  /// Exception to be thrown if LogFileWrite is constructed without a valid pointer to a parent
  class bad_logfile_parent : public std::exception
  {
	  virtual const char* what() const throw()
		{
		  return "LogFileWrite requires non-null LogFile parent";
		}
  };

  /// Callable class for implementing the writing thread
  class LogFileWrite
  {
  public:
	  LogFileWrite(LogFile *parent=NULL);

	  /** This is the functor used by boost::thread.  In this implementation
	   * it simply calls write_thread() for clarity.
	   */
	  void operator()(){write_thread();}

  private:
	  /** This function runs periodically using a QNX timer.  It empties the
	   * log buffers and writes sends their contents to the LogFileWrite::write
	   * function which actually performs the disk write.
	   */
	  void write_thread();
	  /** Write data out to disk
	   * @param log pointer to new data to log
	   * @param headers header to put at top of file.  This paramter is only used
	   * when a file is first created
	   */
	  void write(std::map<std::string, std::queue<std::string> > *log, std::map<std::string, std::string> * headers);

	  /// Pointer to LogFile class which instantiated LogFileWrite
	  LogFile *parent;
	  /// Pulse code used to identify when logfile should write
	  const short LogFile_Pulse_Code;
	  /** mutex used to protect the terminate flag, LogFile::LogFileWrite::terminate
	   * this member must be static because LogFileWrite gets constructed and destructed
	   * several times when LogFile::data_out is initialized, but the copy constructor
	   * for boost::recursive_mutex is private (since it can't be copied).
	   * \todo fix with mutable and copy constructor
	   */
	  static boost::recursive_mutex terminate_mutex;
	  /// terminate flag (static for the same reason as terminate_mutex
	  static bool terminate;
	  /// function to determine if LogFile::LogFileWrite::terminate is true
	  bool check_terminate();
	  /// keep track of open files
	  std::map<std::string, std::fstream*> openFiles;

	  /// This is the slot used by boost::signals2 for terminating the writing thread
	  class do_terminate
	  {
	  public:
		  /// set LogFile::LogFileWrite::terminate to true
		  void operator()();
	  };
  };
};

//template<typename T>
//void LogFile::logData(const std::string& name, const std::vector<T>& data)
//{
//  std::stringstream *dataStr = new std::stringstream();
//
//  boost::posix_time::ptime time(boost::posix_time::microsec_clock::local_time());
//  *dataStr << ((time-startTime).total_milliseconds()) << '\t';
//  if (data.size() > 0)
//  {
//	  for (unsigned int i=0; i<(data.size()); ++i)
//		  *dataStr << boost::lexical_cast<std::string>(data[i]) << '\t';
//  }
//  *dataStr << std::endl;
//  {
//	  boost::mutex::scoped_lock(this->logMutex);
//  	  (*log)[name].push(dataStr->str());
//  }
//  delete dataStr;
//}
//
//template<typename T, std::size_t N>
//void LogFile::logData(const std::string& name, const boost::array<T,N>& data)
//{
//  std::stringstream *dataStr = new std::stringstream();
//
//  boost::posix_time::ptime time(boost::posix_time::microsec_clock::local_time());
//  *dataStr << ((time-startTime).total_milliseconds()) << '\t';
//  if (data.size() > 0)
//  {
//	  for (unsigned int i=0; i<(data.size()); ++i)
//		  *dataStr << boost::lexical_cast<std::string>(data[i]) << '\t';
//  }
//  *dataStr << std::endl;
//  {
//	  boost::mutex::scoped_lock(this->logMutex);
//  	  (*log)[name].push(dataStr->str());
//  }
//  delete dataStr;
//}

template<typename DataContainer>
void LogFile::logData(const std::string& name, const DataContainer& data)
{

	boost::posix_time::ptime time();
	std::string log(boost::lexical_cast<std::string>((boost::posix_time::microsec_clock::local_time() - startTime).total_milliseconds()));
	log += '\t';

	for (typename DataContainer::const_iterator it = data.begin(); it != data.end(); ++it)
	{
		log += boost::lexical_cast<std::string>(*it) + '\t';
	}

	std::stringstream ss;
	ss << log << std::endl;

	{
		boost::mutex::scoped_lock(logMutex);
		(*(this->log))[name].push(ss.str());
	}
}


#endif
