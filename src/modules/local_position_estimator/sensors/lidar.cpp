#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
//
static const uint32_t 		REQ_LIDAR_INIT_COUNT = 10;
static const uint32_t 		LIDAR_TIMEOUT =   1000000; // 1.0 s

void BlockLocalPositionEstimator::lidarInit()
{
	// measure
	Vector<float, n_y_lidar> y;

	if (lidarMeasure(y) != OK) {
		_lidarStats.reset();
	}

	// if finished
	if (_lidarStats.getCount() > REQ_LIDAR_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] lidar init: "
					     "mean %d cm stddev %d cm",
					     int(100 * _lidarStats.getMean()(0)),
					     int(100 * _lidarStats.getStdDev()(0)));
		_sensorTimeout &= ~SENSOR_LIDAR;
		_sensorFault &= ~SENSOR_LIDAR;
	}
}

int BlockLocalPositionEstimator::lidarMeasure(Vector<float, n_y_lidar> &y)
{
	// measure
	float d = _sub_lidar->get().current_distance;
	float eps = 0.01f; // 1 cm
	float min_dist = _sub_lidar->get().min_distance + eps;
	float max_dist = _sub_lidar->get().max_distance - eps;

	// prevent driver from setting min dist below eps
	if (min_dist < eps) {
		min_dist = eps;
	}

	// check for bad data
	if (d > max_dist || d < min_dist) {
		return -1;
	}

	// update stats
	_lidarStats.update(Scalarf(d));
	_time_last_lidar = _timeStamp;
	y.setZero();
	y(0) = (d + _lidar_z_offset.get()) *
	       cosf(_eul(0)) *
	       cosf(_eul(1));
	return OK;
}

void BlockLocalPositionEstimator::lidarCorrect()
{
	// measure lidar
	Vector<float, n_y_lidar> y;

	if (lidarMeasure(y) != OK) { return; }

	// account for leaning
	y(0) = y(0) *
	       cosf(_eul(0)) *
	       cosf(_eul(1));

	// measurement matrix
	Matrix<float, n_y_lidar, n_x> C;
	C.setZero();
	// y = -(z - tz)
	// TODO could add trig to make this an EKF correction
	C(Y_lidar_z, X_z) = -1; // measured altitude, negative down dir.
	C(Y_lidar_z, X_tz) = 1; // measured altitude, negative down dir.

	// use parameter covariance unless sensor provides reasonable value
	SquareMatrix<float, n_y_lidar> R;
	R.setZero();
	float cov = _sub_lidar->get().covariance;

	if (cov < 1.0e-3f) {
		R(0, 0) = _lidar_z_stddev.get() * _lidar_z_stddev.get();

	} else {
		R(0, 0) = cov;
	}

	// residual
	Matrix<float, n_y_lidar, n_y_lidar> S_I = inv<float, n_y_lidar>((C * _P * C.transpose()) + R);
	Vector<float, n_y_lidar> r = y - C * _x;
	_pub_innov.get().hagl_innov = r(0);
	_pub_innov.get().hagl_innov_var = R(0, 0);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_lidar]) {
		if (!(_sensorFault & SENSOR_LIDAR)) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] lidar fault,  beta %5.2f", double(beta));
			_sensorFault |= SENSOR_LIDAR;
		}

		// abort correction
		return;

	} else if (_sensorFault & SENSOR_LIDAR) {
		_sensorFault &= ~SENSOR_LIDAR;
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] lidar OK");
	}

	// kalman filter correction always
	Matrix<float, n_x, n_y_lidar> K = _P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	_P -= K * C * _P;
}

void BlockLocalPositionEstimator::lidarCheckTimeout()
{
	if (_timeStamp - _time_last_lidar > LIDAR_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_LIDAR)) {
			_sensorTimeout |= SENSOR_LIDAR;
			_lidarStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] lidar timeout ");
		}
	}
}
