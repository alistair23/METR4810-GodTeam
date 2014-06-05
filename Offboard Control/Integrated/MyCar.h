#ifndef _INCLUDED_MYCAR_H
#define _INCLUDED_MYCAR_H

#include "Car.h"
#include "Point.h"
#include "Globals.h"

#include <cv.h>		// For Kalman Filter

namespace RaceControl {

class MyCar: public Car {

public:

	MyCar();
	MyCar(Point pos, double dir, double spd, double length = DEFAULT_CAR_LENGTH_PIX,
		double width = DEFAULT_CAR_WIDTH_PIX, double axle_length_ = 0.075 / M_PER_PIX);
	MyCar(const MyCar& c);	// Copy
	const MyCar &MyCar::operator = (const MyCar& c);

	void updateKalman(Point pos, double dir);
	void setRSpeed(double speed);
	void setLSpeed(double speed);
	void step(double seconds);

	double getAxleLength() const;
	double getRSpeed() const;
	double getLSpeed() const;

private:

	double axle_length_;	// Distance between wheels
	double r_speed_;		// Speed of right wheels in pixels/s
	double l_speed_;		// Speed of left wheels in pixels/s

	cv::KalmanFilter kalman_;
};

}

#endif