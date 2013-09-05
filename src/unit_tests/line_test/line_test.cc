#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Line_Trajectory_Tests
#include <boost/test/unit_test.hpp>
#include "control/line.h"
#include <cstdlib>
#include <math.h>
#include "IMU.h"


line l;
BOOST_AUTO_TEST_CASE(Parameters)
{
	BOOST_TEST_MESSAGE("Testing if parameters are set correctly");
	BOOST_CHECK(l.get_x_travel() == 0);
	BOOST_CHECK(l.get_y_travel() == 0);
	BOOST_CHECK(l.get_hover_time() == 0);
	BOOST_CHECK(l.get_speed() == 0);

	double x = round(10.0*rand()/RAND_MAX*15)/10.0+5; // random num between 5 and 20
	double y = round(10.0*rand()/RAND_MAX*15)/10.0+5; // random num between 5 and 20
	double hover_time = 5;
	double speed = round(10.0*rand()/RAND_MAX*9.0)/10.0+1;
	l.set_x_travel(x);
	l.set_y_travel(y);
	l.set_hover_time(hover_time);
	l.set_speed(speed);

	BOOST_CHECK(l.get_x_travel() == x);
	BOOST_CHECK(l.get_y_travel() == y);
	BOOST_CHECK(l.get_speed() == speed);
	BOOST_CHECK(l.get_hover_time() == hover_time);
}

BOOST_AUTO_TEST_CASE(Trajectory)
{
	BOOST_TEST_MESSAGE("Testing trajectory using aligned origins and 0 heading");
	l.reset();
	blas::vector<double> body_travel(blas::zero_vector<double>(3));
	body_travel(0) = l.get_x_travel();
	body_travel(1) = l.get_y_travel();

	BOOST_CHECK(norm_2(body_travel) == l.get_distance());
}

blas::vector<double> IMU::get_ned_position() const
{
	return blas::zero_vector<double>(3);
}

blas::matrix<double> IMU::get_heading_rotation() const
{
	return blas::identity_matrix<double>(3,3);
}
