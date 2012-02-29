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

#include "Debug.h"

Debug::Debug(DEBUG_LEVEL debug_level)
{
	this->debug_level = debug_level;
}

Debug::Debug(const Debug& other)
:debug_level(other.debug_level)
{
	ss << other.ss.rdbuf();
}

Debug::~Debug()
{
	std::string message;
	if (debug_level == WARNING)
		message += "Warning: ";
	else if (debug_level == CRITICAL)
		message += "Critical: ";
	else if (debug_level == MESSAGE)
		message += "Message: ";
// only print debugging messages when NDEBUG is not defined
#if !defined(NDEBUG)
	else if (debug_level == DEBUG)
		message += "Debug: ";
	{
		boost::mutex::scoped_lock lock(cerr_lock);
		std::cerr << message << ss.rdbuf() << std::endl;
	}
#endif
	if (debug_level == WARNING)
	{
		message += ss.str();
		LogFile::getInstance()->logMessage("messages.log", message);
		Debug::warning(message);
	}
	else if (debug_level == CRITICAL)
	{
		message += ss.str();
		LogFile::getInstance()->logMessage("messages.log", message);
		Debug::critical(message);
	}
	else if(debug_level == MESSAGE)
	{
		message += ss.str();
		LogFile::getInstance()->logMessage("messages.log", message);
	}
}

boost::mutex Debug::cerr_lock;
boost::signals2::signal<void (std::string)> Debug::warning;
boost::signals2::signal<void (std::string)> Debug::critical;

Debug& Debug::operator<<(const std::string& s)
{
	ss << s;
	return *this;
}

Debug& Debug::operator<<(const char* c)
{
	ss << c;
	return *this;
}

Debug& Debug::operator<<(const int i)
{
	ss << ' ' << i << ' ';
	return *this;
}

Debug& Debug::operator<<(const unsigned int i)
{
	ss << ' ' << i << ' ';
	return *this;
}

Debug& Debug::operator<<(const unsigned long i)
{
	ss << ' ' << i << ' ';
	return *this;
}

Debug& Debug::operator<<(const double d)
{
	ss << ' ' << d << ' ';
	return *this;
}

Debug& Debug::operator<<(const rapidxml::xml_node<>& node)
{
	ss << node;
	return *this;
}

Debug& Debug::operator<<(std::ios_base& (*pf)(std::ios_base&))
{
	ss << pf;
	return *this;
}

Debug& Debug::operator<<(std::ostream& (*pf)(std::ostream&))
{
	ss << pf;
	return *this;
}

Debug& Debug::operator<<(const std::vector<uint8_t>& v)
{
	ss << "[";
	for (size_t i = 0; i<v.size(); i++)
	{
		ss << static_cast<int>(v[i]);
		if (i < v.size() - 1)
			ss << ", ";
	}
	ss << "]";
	return *this;
}
