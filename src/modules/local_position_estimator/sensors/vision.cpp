#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor to initialize.
// This is a vision based position measurement so we assume
// as soon as we get one measurement it is initialized.
static const uint32_t 		REQ_VISION_INIT_COUNT = 1;

// We don't want to deinitialize it because
// this will throw away a correction before it starts using the data so we
// set the timeout to 0.5 seconds
static const uint32_t 		VISION_TIMEOUT =    500000;	// 0.5 s

void BlockLocalPositionEstimator::visionInit()
{
	// measure
	Vector<float, n_y_vision> y;

	if (visionMeasure(y) != OK) {
		_visionStats.reset();
		return;
	}

	// increament sums for mean
	if (_visionStats.getCount() > REQ_VISION_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position init: "
					     "%5.2f %5.2f %5.2f m std %5.2f %5.2f %5.2f m",
					     double(_visionStats.getMean()(0)),
					     double(_visionStats.getMean()(1)),
					     double(_visionStats.getMean()(2)),
					     double(_visionStats.getStdDev()(0)),
					     double(_visionStats.getStdDev()(1)),
					     double(_visionStats.getStdDev()(2)));
		_sensorTimeout &= ~SENSOR_VISION;
		_sensorFault &= ~SENSOR_VISION;

		if (!_map_ref.init_done && _sub_vision_pos.get().xy_global) {
			// initialize global origin using the visual estimator reference
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] global origin init (vision) : lat %6.2f lon %6.2f alt %5.1f m",
						     double(_sub_vision_pos.get().ref_lat), double(_sub_vision_pos.get().ref_lon), double(_sub_vision_pos.get().ref_alt));
			map_projection_init(&_map_ref, _sub_vision_pos.get().ref_lat, _sub_vision_pos.get().ref_lon);
		}

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOrigin = _sub_vision_pos.get().z_global ? _sub_vision_pos.get().ref_alt : 0.0f;
		}
	}
}

int BlockLocalPositionEstimator::visionMeasure(Vector<float, n_y_vision> &y)
{
	y.setZero();
	y(Y_vision_x) = _sub_vision_pos.get().x;
	y(Y_vision_y) = _sub_vision_pos.get().y;
	y(Y_vision_z) = _sub_vision_pos.get().z;
	_visionStats.update(y);
	_time_last_vision_p = _sub_vision_pos.get().timestamp;
	return OK;
}

void BlockLocalPositionEstimator::visionCorrect()
{
	// measure
	Vector<float, n_y_vision> y;

	if (visionMeasure(y) != OK) { return; }

	// vision measurement matrix, measures position
	Matrix<float, n_y_vision, n_x> C;
	C.setZero();
	C(Y_vision_x, X_x) = 1;
	C(Y_vision_y, X_y) = 1;
	C(Y_vision_z, X_z) = 1;

	// noise matrix
	Matrix<float, n_y_vision, n_y_vision> R;
	R.setZero();

	// use error estimates from vision topic if available
	if (_sub_vision_pos.get().eph > _vision_xy_stddev.get()) {
		R(Y_vision_x, Y_vision_x) = _sub_vision_pos.get().eph * _sub_vision_pos.get().eph;
		R(Y_vision_y, Y_vision_y) = _sub_vision_pos.get().eph * _sub_vision_pos.get().eph;

	} else {
		R(Y_vision_x, Y_vision_x) = _vision_xy_stddev.get() * _vision_xy_stddev.get();
		R(Y_vision_y, Y_vision_y) = _vision_xy_stddev.get() * _vision_xy_stddev.get();
	}

	if (_sub_vision_pos.get().epv > _vision_z_stddev.get()) {
		R(Y_vision_z, Y_vision_z) = _sub_vision_pos.get().epv * _sub_vision_pos.get().epv;

	} else {
		R(Y_vision_z, Y_vision_z) = _vision_z_stddev.get() * _vision_z_stddev.get();
	}

	// vision delayed x
	uint8_t i_hist = 0;

	float vision_delay = (_timeStamp - _sub_vision_pos.get().timestamp) * 1e-6f; // measurement delay in seconds

	if (vision_delay < 0.0f) { vision_delay = 0.0f; }

	// use auto-calculated delay from measurement if parameter is set to zero
	if (getDelayPeriods(_vision_delay.get() > 0.0f ? _vision_delay.get() : vision_delay, &i_hist)  < 0) { return; }

	//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision delay : %0.2f ms", double(vision_delay * 1000.0f));

	Vector<float, n_x> x0 = _xDelay.get(i_hist);

	// residual
	Matrix<float, n_y_vision, n_y_vision> S_I = inv<float, n_y_vision>((C * _P * C.transpose()) + R);
	Matrix<float, n_y_vision, 1> r = y - C * x0;

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_vision]) {
		if (!(_sensorFault & SENSOR_VISION)) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position fault, beta %5.2f", double(beta));
			_sensorFault |= SENSOR_VISION;
		}

	} else if (_sensorFault & SENSOR_VISION) {
		_sensorFault &= ~SENSOR_VISION;
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vision position OK");
	}

	// kalman filter correction if no fault
	if (!(_sensorFault & SENSOR_VISION)) {
		Matrix<float, n_x, n_y_vision> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		_x += dx;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::visionCheckTimeout()
{
	if (_timeStamp - _time_last_vision_p > VISION_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_VISION)) {
			_sensorTimeout |= SENSOR_VISION;
			_visionStats.reset();
			mavlink_log_critical(&mavlink_log_pub, "[lpe] vision position timeout ");
		}
	}
}
