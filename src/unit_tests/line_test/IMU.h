#ifndef IMU_H_
#define IMU_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;

class IMU
{
public:
	static IMU* getInstance();
	blas::vector<double> get_ned_position() const;
	blas::matrix<double> get_heading_rotation() const;
private:
	IMU();
	static IMU* _instance;
};
#endif
