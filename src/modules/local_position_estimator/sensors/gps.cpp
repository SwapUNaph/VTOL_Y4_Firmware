#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t 		REQ_GPS_INIT_COUNT = 10;
static const uint32_t 		GPS_TIMEOUT =      1000000; // 1.0 s

void BlockLocalPositionEstimator::gpsInit()
{
	// check for good gps signal
	uint8_t nSat = _sub_gps.get().satellites_used;
	float eph = _sub_gps.get().eph;
	float epv = _sub_gps.get().epv;
	uint8_t fix_type = _sub_gps.get().fix_type;

	if (
		nSat < 6 ||
		eph > _gps_eph_max.get() ||
		epv > _gps_epv_max.get() ||
		fix_type < 3
	) {
		_gpsStats.reset();
		return;
	}

	// measure
	Vector<double, n_y_gps> y;

	if (gpsMeasure(y) != OK) {
		_gpsStats.reset();
		return;
	}

	// if finished
	if (_gpsStats.getCount() > REQ_GPS_INIT_COUNT) {
		// get mean gps values
		double gpsLat = _gpsStats.getMean()(0);
		double gpsLon = _gpsStats.getMean()(1);
		float gpsAlt = _gpsStats.getMean()(2);

		_sensorTimeout &= ~SENSOR_GPS;
		_sensorFault &= ~SENSOR_GPS;
		_gpsStats.reset();

		if (!_receivedGps) {
			// this is the first time we have received gps
			_receivedGps = true;

			// note we subtract X_z which is in down directon so it is
			// an addition
			_gpsAltOrigin = gpsAlt + _x(X_z);

			// find lat, lon of current origin by subtracting x and y
			// if not using vision position since vision will
			// have it's own origin, not necessarily where vehicle starts
			if (!_map_ref.init_done && !(_fusion.get() & FUSE_VIS_POS)) {
				double gpsLatOrigin = 0;
				double gpsLonOrigin = 0;
				// reproject at current coordinates
				map_projection_init(&_map_ref, gpsLat, gpsLon);
				// find origin
				map_projection_reproject(&_map_ref, -_x(X_x), -_x(X_y), &gpsLatOrigin, &gpsLonOrigin);
				// reinit origin
				map_projection_init(&_map_ref, gpsLatOrigin, gpsLonOrigin);

				// always override alt origin on first GPS to fix
				// possible baro offset in global altitude at init
				_altOrigin = _gpsAltOrigin;
				_altOriginInitialized = true;

				mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] global origin init (gps) : lat %6.2f lon %6.2f alt %5.1f m",
							     gpsLatOrigin, gpsLonOrigin, double(_gpsAltOrigin));
			}

			PX4_INFO("[lpe] gps init "
				 "lat %6.2f lon %6.2f alt %5.1f m",
				 gpsLat,
				 gpsLon,
				 double(gpsAlt));
		}
	}
}

int BlockLocalPositionEstimator::gpsMeasure(Vector<double, n_y_gps> &y)
{
	// gps measurement
	y.setZero();
	y(0) = _sub_gps.get().lat * 1e-7;
	y(1) = _sub_gps.get().lon * 1e-7;
	y(2) = _sub_gps.get().alt * 1e-3;
	y(3) = _sub_gps.get().vel_n_m_s;
	y(4) = _sub_gps.get().vel_e_m_s;
	y(5) = _sub_gps.get().vel_d_m_s;

	// increament sums for mean
	_gpsStats.update(y);
	_time_last_gps = _timeStamp;
	return OK;
}

void BlockLocalPositionEstimator::gpsCorrect()
{
	// measure
	Vector<double, n_y_gps> y_global;

	if (gpsMeasure(y_global) != OK) { return; }

	// gps measurement in local frame
	double  lat = y_global(0);
	double  lon = y_global(1);
	float  alt = y_global(2);
	float px = 0;
	float py = 0;
	float pz = -(alt - _gpsAltOrigin);
	map_projection_project(&_map_ref, lat, lon, &px, &py);
	Vector<float, 6> y;
	y.setZero();
	y(0) = px;
	y(1) = py;
	y(2) = pz;
	y(3) = y_global(3);
	y(4) = y_global(4);
	y(5) = y_global(5);

	// gps measurement matrix, measures position and velocity
	Matrix<float, n_y_gps, n_x> C;
	C.setZero();
	C(Y_gps_x, X_x) = 1;
	C(Y_gps_y, X_y) = 1;
	C(Y_gps_z, X_z) = 1;
	C(Y_gps_vx, X_vx) = 1;
	C(Y_gps_vy, X_vy) = 1;
	C(Y_gps_vz, X_vz) = 1;

	// gps covariance matrix
	SquareMatrix<float, n_y_gps> R;
	R.setZero();

	// default to parameter, use gps cov if provided
	float var_xy = _gps_xy_stddev.get() * _gps_xy_stddev.get();
	float var_z = _gps_z_stddev.get() * _gps_z_stddev.get();
	float var_vxy = _gps_vxy_stddev.get() * _gps_vxy_stddev.get();
	float var_vz = _gps_vz_stddev.get() * _gps_vz_stddev.get();

	// if field is not below minimum, set it to the value provided
	if (_sub_gps.get().eph > _gps_xy_stddev.get()) {
		var_xy = _sub_gps.get().eph * _sub_gps.get().eph;
	}

	if (_sub_gps.get().epv > _gps_z_stddev.get()) {
		var_z = _sub_gps.get().epv * _sub_gps.get().epv;
	}

	float gps_s_stddev =  _sub_gps.get().s_variance_m_s;

	if (gps_s_stddev > _gps_vxy_stddev.get()) {
		var_vxy = gps_s_stddev * gps_s_stddev;
	}

	if (gps_s_stddev > _gps_vz_stddev.get()) {
		var_vz = gps_s_stddev * gps_s_stddev;
	}


	R(0, 0) = var_xy;
	R(1, 1) = var_xy;
	R(2, 2) = var_z;
	R(3, 3) = var_vxy;
	R(4, 4) = var_vxy;
	R(5, 5) = var_vz;

	// get delayed x
	uint8_t i_hist = 0;

	if (getDelayPeriods(_gps_delay.get(), &i_hist)  < 0) { return; }

	Vector<float, n_x> x0 = _xDelay.get(i_hist);

	// residual
	Vector<float, n_y_gps> r = y - C * x0;

	for (int i = 0; i < 6; i ++) {
		_pub_innov.get().vel_pos_innov[i] = r(i);
		_pub_innov.get().vel_pos_innov_var[i] = R(i, i);
	}

	Matrix<float, n_y_gps, n_y_gps> S_I = inv<float, 6>(C * _P * C.transpose() + R);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	// artifically increase beta threshhold to prevent fault during landing
	float beta_thresh = 1e2f;

	if (beta / BETA_TABLE[n_y_gps] > beta_thresh) {
		if (!(_sensorFault & SENSOR_GPS)) {
			mavlink_log_critical(&mavlink_log_pub, "[lpe] gps fault %3g %3g %3g %3g %3g %3g",
					     double(r(0)*r(0) / S_I(0, 0)),  double(r(1)*r(1) / S_I(1, 1)), double(r(2)*r(2) / S_I(2, 2)),
					     double(r(3)*r(3) / S_I(3, 3)),  double(r(4)*r(4) / S_I(4, 4)), double(r(5)*r(5) / S_I(5, 5)));
			_sensorFault |= SENSOR_GPS;
		}

	} else if (_sensorFault & SENSOR_GPS) {
		_sensorFault &= ~SENSOR_GPS;
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] GPS OK");
	}

	// kalman filter correction always for GPS
	Matrix<float, n_x, n_y_gps> K = _P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	_P -= K * C * _P;
}

void BlockLocalPositionEstimator::gpsCheckTimeout()
{
	if (_timeStamp - _time_last_gps > GPS_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_GPS)) {
			_sensorTimeout |= SENSOR_GPS;
			_gpsStats.reset();
			mavlink_log_critical(&mavlink_log_pub, "[lpe] GPS timeout ");
		}
	}
}
