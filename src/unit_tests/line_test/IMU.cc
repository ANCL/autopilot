#include "IMU.h"
#include <cstdlib>



IMU* IMU::_instance = NULL;

IMU* IMU::getInstance()
{
	if (!_instance)
		_instance = new IMU;
	return _instance;
}

IMU::IMU()
{

}


