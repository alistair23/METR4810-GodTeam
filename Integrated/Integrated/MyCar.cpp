#include "MyCar.h"

#define _USE_MATH_DEFINES

#include <math.h>
#include "CommonFunctions.h"

using namespace RaceControl;

MyCar::MyCar():
	Car(),
	axle_length_(0.075 / M_PER_PIX),
	l_speed_(0),
	r_speed_(0),
	kalman_(cv::KalmanFilter(4, 2, 0))
{
	kalman_.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
	cv::Mat_<float> measurement(2,1);
	measurement.setTo(cv::Scalar(0));
	kalman_.statePre.at<float>(0) = pos_.x;
	kalman_.statePre.at<float>(1) = pos_.y;
	kalman_.statePre.at<float>(2) = 0;
	kalman_.statePre.at<float>(3) = 0;
	cv::setIdentity(kalman_.measurementMatrix);
	cv::setIdentity(kalman_.processNoiseCov, cv::Scalar::all(1e-4));
	cv::setIdentity(kalman_.measurementNoiseCov, cv::Scalar::all(1e-1));
	cv::setIdentity(kalman_.errorCovPost, cv::Scalar::all(.1));
}

MyCar::MyCar(Point pos, double dir, double spd, double length, double width, double axle_length) :
	Car(pos, dir, spd, length, width),
	axle_length_(axle_length),
	l_speed_(0),
	r_speed_(0),
	kalman_(cv::KalmanFilter(4, 2, 0))
{
	kalman_.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
	cv::Mat_<float> measurement(2,1);
	measurement.setTo(cv::Scalar(0));
	kalman_.statePre.at<float>(0) = pos_.x;
	kalman_.statePre.at<float>(1) = pos_.y;
	kalman_.statePre.at<float>(2) = 0;
	kalman_.statePre.at<float>(3) = 0;
	cv::setIdentity(kalman_.measurementMatrix);
	cv::setIdentity(kalman_.processNoiseCov, cv::Scalar::all(1e-4));
	cv::setIdentity(kalman_.measurementNoiseCov, cv::Scalar::all(1e-1));
	cv::setIdentity(kalman_.errorCovPost, cv::Scalar::all(.1));
}

// Copy
MyCar::MyCar(const MyCar& c) {

	// allocate variables
	MyCar();

	// copy values
	operator = (c);
}

// Overload equals operator
const MyCar &MyCar::operator = (const MyCar& c) {

	// Copy stuff
	pos_ = c.getPos();
	dir_ = c.getDir();
	spd_ = c.getSpd();
	update_time_ = c.getUpdateTime();
	axle_length_ = c.getAxleLength();
	r_speed_ = c.getRSpeed();
	l_speed_ = c.getLSpeed();
	width_ = c.getWidth();
	length_ = c.getLength();
	return *this;
}

void MyCar::updateKalman(Point pos, double dir) { 

	// First predict, to update the internal statePre variable
	cv::Mat prediction = kalman_.predict();
	cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
             
	// Get mouse point
	cv::Mat_<float> measurement(2,1);
	measurement.setTo(cv::Scalar(0));
	measurement(0) = pos.x;
	measurement(1) = pos.y;
             
	cv::Point measPt(measurement(0),measurement(1));
 
	// The "correct" phase that is going to use the predicted value and our measurement
	cv::Mat estimated = kalman_.correct(measurement);
	cv::Point statePt(estimated.at<float>(0),estimated.at<float>(1));

	pos_.x = statePt.x;
	pos_.y = statePt.y;
	dir_ = dir;
	update_time_ = time_now();
}

void MyCar::setRSpeed(double speed) {
	r_speed_ = speed;
}

void MyCar::setLSpeed(double speed) {
	l_speed_ = speed;
}

// Assumes 2-wheel differential steering. Calculate 
// new position after moving a certain amount of time.
// http://rossum.sourceforge.net/papers/DiffSteer/
void MyCar::step(double seconds) {
	double speed_diff = l_speed_ - r_speed_;
	
	// Special case for when left wheel speed == right wheel speed
	// Needed to avoid division by zero
	if (speed_diff == 0) {
		pos_.x += r_speed_ * seconds * cos(dir_);
		pos_.y += r_speed_ * seconds * sin(dir_);
	}
	else {
		double speed_sum = r_speed_ + l_speed_;
		double old_dir = dir_;
		dir_ += speed_diff * seconds / axle_length_;
		double a = axle_length_ * speed_sum * 0.5 / speed_diff;
		pos_.x += a * (sin(dir_) - sin(old_dir));
		pos_.y -= a * (cos(dir_) - cos(old_dir));

		// Ensure dir_ is between -pi and pi rad
		while (dir_ > M_PI)
			dir_ -= 2 * M_PI;
		while (dir_ < -M_PI)
			dir_ += 2 * M_PI;
	}
}

double MyCar::getAxleLength() const {
	return axle_length_;
}

double MyCar::getRSpeed() const {
	return r_speed_;
}

double MyCar::getLSpeed() const {
	return l_speed_;
}