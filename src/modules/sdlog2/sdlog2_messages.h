/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sdlog2_messages.h
 *
 * Log messages and structures definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Roman Bapst <roman@px4.io>
 */

#ifndef SDLOG2_MESSAGES_H_
#define SDLOG2_MESSAGES_H_

#include "sdlog2_format.h"

/* define message formats */

#pragma pack(push, 1)
/* --- ATT - ATTITUDE --- */
#define LOG_ATT_MSG 2
struct log_ATT_s {
	float q_w;
	float q_x;
	float q_y;
	float q_z;
	float roll;
	float pitch;
	float yaw;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
};

/* --- ATSP - ATTITUDE SET POINT --- */
#define LOG_ATSP_MSG 3
struct log_ATSP_s {
	float roll_sp;
	float pitch_sp;
	float yaw_sp;
	float thrust_sp;
	float q_w;
	float q_x;
	float q_y;
	float q_z;
};

/* --- IMU - IMU SENSORS --- */
#define LOG_IMU_MSG 4
#define LOG_IMU1_MSG 22
#define LOG_IMU2_MSG 23
struct log_IMU_s {
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	float temp_acc;
	float temp_gyro;
	float temp_mag;
};

/* --- SENS - OTHER SENSORS --- */
#define LOG_SENS_MSG 5
struct log_SENS_s {
	float baro_pres;
	float baro_alt;
	float baro_temp;
};

/* --- LPOS - LOCAL POSITION --- */
#define LOG_LPOS_MSG 6
struct log_LPOS_s {
	float x;
	float y;
	float z;
	float ground_dist;
	float ground_dist_rate;
	float vx;
	float vy;
	float vz;
	int32_t ref_lat;
	int32_t ref_lon;
	float ref_alt;
	uint8_t pos_flags;
	uint8_t ground_dist_flags;
	float eph;
	float epv;
};

/* --- LPSP - LOCAL POSITION SETPOINT --- */
#define LOG_LPSP_MSG 7
struct log_LPSP_s {
	float x;
	float y;
	float z;
	float yaw;
	float vx;
	float vy;
	float vz;
	float acc_x;
	float acc_y;
	float acc_z;
};

/* --- GPS - GPS POSITION --- */
#define LOG_GPS_MSG 8
#define LOG_DGPS_MSG 58
struct log_GPS_s {
	uint64_t gps_time;
	uint8_t fix_type;
	float eph;
	float epv;
	int32_t lat;
	int32_t lon;
	float alt;
	float vel_n;
	float vel_e;
	float vel_d;
	float cog;
	uint8_t sats;
	uint16_t snr_mean;
	uint16_t noise_per_ms;
	uint16_t jamming_indicator;
};

/* --- ATTC - ATTITUDE CONTROLS (ACTUATOR_0 CONTROLS)--- */
#define LOG_ATTC_MSG 9
#define LOG_ATC1_MSG 46
struct log_ATTC_s {
	float roll;
	float pitch;
	float yaw;
	float thrust;
};

/* --- STAT - VEHICLE STATE --- */
#define LOG_STAT_MSG 10
struct log_STAT_s {
	uint8_t main_state;
	uint8_t nav_state;
	uint8_t arming_state;
	uint8_t failsafe;
	uint8_t is_rot_wing;
};

/* --- RC - RC INPUT CHANNELS --- */
#define LOG_RC_MSG 11
struct log_RC_s {
	float channel[12];
	uint8_t rssi;
	uint8_t channel_count;
	uint8_t signal_lost;
	uint32_t frame_drop;
};

/* --- OUT - ACTUATOR OUTPUT --- */
#define LOG_OUT0_MSG 12
struct log_OUT_s {
	float output[8];
};

/* --- AIRS - AIRSPEED --- */
#define LOG_AIRS_MSG 13
struct log_AIRS_s {
	float indicated_airspeed_m_s;
	float true_airspeed_m_s;
	float true_airspeed_unfiltered_m_s;
	float air_temperature_celsius;
	float confidence;
};

/* --- ARSP - ATTITUDE RATE SET POINT --- */
#define LOG_ARSP_MSG 14
struct log_ARSP_s {
	float roll_rate_sp;
	float pitch_rate_sp;
	float yaw_rate_sp;
};

/* --- FLOW - OPTICAL FLOW --- */
#define LOG_FLOW_MSG 15
struct log_FLOW_s {
	uint8_t sensor_id;
	float pixel_flow_x_integral;
	float pixel_flow_y_integral;
	float gyro_x_rate_integral;
	float gyro_y_rate_integral;
	float gyro_z_rate_integral;
	float ground_distance_m;
	uint32_t integration_timespan;
	uint32_t time_since_last_sonar_update;
	uint16_t frame_count_since_last_readout;
	int16_t gyro_temperature;
	uint8_t	quality;
};

/* --- GPOS - GLOBAL POSITION ESTIMATE --- */
#define LOG_GPOS_MSG 16
struct log_GPOS_s {
	int32_t lat;
	int32_t lon;
	float alt;
	float vel_n;
	float vel_e;
	float vel_d;
	float eph;
	float epv;
	float terrain_alt;
};

/* --- GPSP - GLOBAL POSITION SETPOINT --- */
#define LOG_GPSP_MSG 17
struct log_GPSP_s {
	int32_t lat;
	int32_t lon;
	float alt;
	float yaw;
	uint8_t type;
	float loiter_radius;
	int8_t loiter_direction;
	float pitch_min;
};

/* --- ESC - ESC STATE --- */
#define LOG_ESC_MSG 18
struct log_ESC_s {
	uint16_t counter;
	uint8_t  esc_count;
	uint8_t  esc_connectiontype;
	uint8_t  esc_num;
	uint16_t esc_address;
	uint16_t esc_version;
	float    esc_voltage;
	float    esc_current;
	int32_t  esc_rpm;
	float    esc_temperature;
	float    esc_setpoint;
	uint16_t esc_setpoint_raw;
};

/* --- GVSP - GLOBAL VELOCITY SETPOINT --- */
#define LOG_GVSP_MSG 19
struct log_GVSP_s {
	float vx;
	float vy;
	float vz;
};

/* --- BATT - BATTERY --- */
#define LOG_BATT_MSG 20
struct log_BATT_s {
	float voltage;
	float voltage_filtered;
	float current;
	float current_filtered;
	float discharged;
	float remaining;
	float scale;
	uint8_t warning;
};

/* --- DIST - RANGE SENSOR DISTANCE --- */
#define LOG_DIST_MSG 21
struct log_DIST_s {
	uint8_t id;
	uint8_t type;
	uint8_t orientation;
	float current_distance;
	float covariance;
};

/* LOG IMU1 and IMU2 MSGs consume IDs 22 and 23 */


/* --- PWR - ONBOARD POWER SYSTEM --- */
#define LOG_PWR_MSG 24
struct log_PWR_s {
	float peripherals_5v;
	float servo_rail_5v;
	float servo_rssi;
	uint8_t usb_ok;
	uint8_t brick_ok;
	uint8_t servo_ok;
	uint8_t low_power_rail_overcurrent;
	uint8_t high_power_rail_overcurrent;
};

/* --- MOCP - MOCAP ATTITUDE AND POSITION --- */
#define LOG_MOCP_MSG 25
struct log_MOCP_s {
	float qw;
	float qx;
	float qy;
	float qz;
	float x;
	float y;
	float z;
};

/* --- GS0A - GPS SNR #0, SAT GROUP A --- */
#define LOG_GS0A_MSG 26
struct log_GS0A_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- GS0B - GPS SNR #0, SAT GROUP B --- */
#define LOG_GS0B_MSG 27
struct log_GS0B_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- GS1A - GPS SNR #1, SAT GROUP A --- */
#define LOG_GS1A_MSG 28
struct log_GS1A_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- GS1B - GPS SNR #1, SAT GROUP B --- */
#define LOG_GS1B_MSG 29
struct log_GS1B_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- TECS - TECS STATUS --- */
#define LOG_TECS_MSG 30
struct log_TECS_s {
	float altitudeSp;
	float altitudeFiltered;
	float flightPathAngleSp;
	float flightPathAngle;
	float airspeedSp;
	float airspeedFiltered;
	float airspeedDerivativeSp;
	float airspeedDerivative;
	float totalEnergyError;
	float totalEnergyRateError;
	float energyDistributionError;
	float energyDistributionRateError;
	float pitch_integ;
	float throttle_integ;

	uint8_t mode;
};

/* --- WIND - WIND ESTIMATE --- */
#define LOG_WIND_MSG 31
struct log_WIND_s {
	float x;
	float y;
	float cov_x;
	float cov_y;
};

/* --- EST0 - ESTIMATOR STATUS --- */
#define LOG_EST0_MSG 32
struct log_EST0_s {
	float s[12];
	uint8_t n_states;
	uint8_t nan_flags;
	uint16_t fault_flags;
	uint8_t timeout_flags;
};

/* --- EST1 - ESTIMATOR STATUS --- */
#define LOG_EST1_MSG 33
struct log_EST1_s {
	float s[16];
};

/* --- EST2 - ESTIMATOR STATUS --- */
#define LOG_EST2_MSG 34
struct log_EST2_s {
	float cov[12];
	uint16_t gps_check_fail_flags;
	uint16_t control_mode_flags;
	uint8_t health_flags;
	uint16_t innov_test_flags;
};

/* --- EST3 - ESTIMATOR STATUS --- */
#define LOG_EST3_MSG 35
struct log_EST3_s {
    float cov[16];
};

/* --- TEL0..3 - TELEMETRY STATUS --- */
#define LOG_TEL0_MSG 36
#define LOG_TEL1_MSG 37
#define LOG_TEL2_MSG 38
#define LOG_TEL3_MSG 39
struct log_TEL_s {
	uint8_t rssi;
	uint8_t remote_rssi;
	uint8_t noise;
	uint8_t remote_noise;
	uint16_t rxerrors;
	uint16_t fixed;
	uint8_t txbuf;
	uint64_t heartbeat_time;
};

/* --- VISN - VISION POSITION --- */
#define LOG_VISN_MSG 40
struct log_VISN_s {
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
	float qw;
	float qx;
	float qy;
	float qz;
};

/* --- ENCODERS - ENCODER DATA --- */
#define LOG_ENCD_MSG 41
struct log_ENCD_s {
	int64_t cnt0;
	float vel0;
	int64_t cnt1;
	float vel1;
};

/* --- AIR SPEED SENSORS - DIFF. PRESSURE --- */
#define LOG_AIR1_MSG 42

/* --- VTOL - VTOL VEHICLE STATUS */
#define LOG_VTOL_MSG 43
struct log_VTOL_s {
	float airspeed_tot;
	uint8_t rw_mode;
	uint8_t trans_mode;
	uint8_t failsafe_mode;
};

/* --- TIMESYNC - TIME SYNCHRONISATION OFFSET */
#define LOG_TSYN_MSG 44
struct log_TSYN_s {
	uint64_t time_offset;
};

/* --- MACS - MULTIROTOR ATTITUDE CONTROLLER STATUS */
#define LOG_MACS_MSG 45
struct log_MACS_s {
	float roll_rate_integ;
	float pitch_rate_integ;
	float yaw_rate_integ;
};

/* WARNING: ID 46 is already in use for ATTC1 */

/* --- CONTROL STATE --- */
#define LOG_CTS_MSG 47
struct log_CTS_s {
	float vx_body;
	float vy_body;
	float vz_body;
	float airspeed;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
};

/* --- EST4 - ESTIMATOR INNOVATIONS --- */
#define LOG_EST4_MSG 48
struct log_EST4_s {
    float s[15];
};

/* --- EST5 - ESTIMATOR INNOVATIONS --- */
#define LOG_EST5_MSG 49
struct log_EST5_s {
    float s[12];
};

#define LOG_OUT1_MSG 50

/* --- EKF2 REPLAY Part 1 --- */
#define LOG_RPL1_MSG 51
struct log_RPL1_s {
	uint64_t time_ref;
	float gyro_integral_dt;
	float accelerometer_integral_dt;
	uint64_t magnetometer_timestamp;
	uint64_t baro_timestamp;
	float gyro_x_rad;
	float gyro_y_rad;
	float gyro_z_rad;
	float accelerometer_x_m_s2;
	float accelerometer_y_m_s2;
	float accelerometer_z_m_s2;
	float magnetometer_x_ga;
	float magnetometer_y_ga;
	float magnetometer_z_ga;
	float baro_alt_meter;
};
/* --- EKF2 REPLAY Part 2 --- */
#define LOG_RPL2_MSG 52
struct log_RPL2_s {
	uint64_t time_pos_usec;
	uint64_t time_vel_usec;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	uint8_t fix_type;
	uint8_t nsats;
	float eph;
	float epv;
	float sacc;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	bool vel_ned_valid;
};

/* --- EST6 - ESTIMATOR INNOVATIONS --- */
#define LOG_EST6_MSG 53
struct log_EST6_s {
    float s[6];
};

/* --- EKF2 REPLAY Part 3 --- */
#define LOG_RPL3_MSG 54
struct log_RPL3_s {
	uint64_t time_flow_usec;
	float flow_integral_x;
	float flow_integral_y;
	float gyro_integral_x;
	float gyro_integral_y;
	uint32_t flow_time_integral;
	uint8_t flow_quality;
};

/* --- CAMERA TRIGGER --- */
#define LOG_CAMT_MSG 55
struct log_CAMT_s {
	uint64_t timestamp;
	uint32_t seq;
};

/* --- EKF2 REPLAY Part 4 --- */
#define LOG_RPL4_MSG 56
struct log_RPL4_s {
	uint64_t time_rng_usec;
	float range_to_ground;
};

/* --- LAND DETECTOR --- */
#define LOG_LAND_MSG 57
struct log_LAND_s {
	uint8_t landed;
};

/* 58 used for DGPS message
 shares struct with GPS MSG 8*/

/* --- EKF2 REPLAY Part 6 --- */
#define LOG_RPL6_MSG 59
struct log_RPL6_s {
	uint64_t time_airs_usec;
	float indicated_airspeed_m_s;
	float true_airspeed_m_s;
};

/* --- EKF2 REPLAY Part 5 --- */
#define LOG_RPL5_MSG 60
struct log_RPL5_s {
	uint64_t time_ev_usec;
	float x;
	float y;
	float z;
	float q0;
	float q1;
	float q2;
	float q3;
	float pos_err;
	float ang_err;
};

/* --- SYSTEM LOAD --- */
#define LOG_LOAD_MSG 61
struct log_LOAD_s {
	float cpu_load;
};

/* --- DPRS - DIFFERENTIAL PRESSURE --- */
#define LOG_DPRS_MSG 62
struct log_DPRS_s {
	uint64_t error_count;
	float differential_pressure_raw_pa;
	float differential_pressure_filtered_pa;
	float max_differential_pressure_pa;
	float temperature;
};

/* --- LOW STACK --- */
#define LOG_STCK_MSG 63
struct log_STCK_s {
	char task_name[16];
	uint16_t stack_free;
};

/********** SYSTEM MESSAGES, ID > 0x80 **********/

/* --- TIME - TIME STAMP --- */
#define LOG_TIME_MSG 129
struct log_TIME_s {
	uint64_t t;
};

/* --- VER - VERSION --- */
#define LOG_VER_MSG 130
struct log_VER_s {
	char arch[16];
	char fw_git[64];
};

/* --- PARM - PARAMETER --- */
#define LOG_PARM_MSG 131
struct log_PARM_s {
	char name[64];
	float value;
};
#pragma pack(pop)

// the lower type of initialisation is not supported in C++
#ifndef __cplusplus

/* construct list of all message formats */
static const struct log_format_s log_formats[] = {
	/* business-level messages, ID < 0x80 */
	LOG_FORMAT(ATT, "ffffffffff",	"qw,qx,qy,qz,Roll,Pitch,Yaw,RollRate,PitchRate,YawRate"),
	LOG_FORMAT(ATSP, "ffffffff",		"RollSP,PitchSP,YawSP,ThrustSP,qw,qx,qy,qz"),
	LOG_FORMAT_S(IMU, IMU, "ffffffffffff",		"AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM"),
	LOG_FORMAT_S(IMU1, IMU, "ffffffffffff",		"AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM"),
	LOG_FORMAT_S(IMU2, IMU, "ffffffffffff",		"AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM"),
	LOG_FORMAT_S(SENS, SENS, "fff",		"BaroPres,BaroAlt,BaroTemp"),
	LOG_FORMAT_S(AIR1, SENS, "fffff",	"BaroPa,BaroAlt,BaroTmp,DiffPres,DiffPresF"),
	LOG_FORMAT(LPOS, "ffffffffLLfBBff",	"X,Y,Z,Dist,DistR,VX,VY,VZ,RLat,RLon,RAlt,PFlg,GFlg,EPH,EPV"),
	LOG_FORMAT(LPSP, "ffffffffff",		"X,Y,Z,Yaw,VX,VY,VZ,AX,AY,AZ"),
	LOG_FORMAT(GPS, "QBffLLfffffBHHH",	"GPSTime,Fix,EPH,EPV,Lat,Lon,Alt,VelN,VelE,VelD,Cog,nSat,SNR,N,J"),
	LOG_FORMAT_S(DGPS, GPS,	 "QBffLLfffffBHHH",	"GPSTime,Fix,EPH,EPV,Lat,Lon,Alt,VelN,VelE,VelD,Cog,nSat,SNR,N,J"),
	LOG_FORMAT_S(ATTC, ATTC, "ffff",		"Roll,Pitch,Yaw,Thrust"),
	LOG_FORMAT_S(ATC1, ATTC, "ffff",		"Roll,Pitch,Yaw,Thrust"),
	LOG_FORMAT(STAT, "BBBBB",		"MainState,NavState,ArmS,Failsafe,IsRotWing"),
	LOG_FORMAT(VTOL, "fBBB",		"Arsp,RwMode,TransMode,Failsafe"),
	LOG_FORMAT(CTS, "fffffff", "Vx_b,Vy_b,Vz_b,Vinf,P,Q,R"),
	LOG_FORMAT(RC, "ffffffffffffBBBL",		"C0,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,RSSI,CNT,Lost,Drop"),
	LOG_FORMAT_S(OUT0, OUT, "ffffffff",		"Out0,Out1,Out2,Out3,Out4,Out5,Out6,Out7"),
	LOG_FORMAT_S(OUT1, OUT, "ffffffff",		"Out0,Out1,Out2,Out3,Out4,Out5,Out6,Out7"),
	LOG_FORMAT(AIRS, "fffff",			"IAS,TAS,TASraw,Temp,Confidence"),
	LOG_FORMAT(ARSP, "fff",			"RollRateSP,PitchRateSP,YawRateSP"),
	LOG_FORMAT(FLOW, "BffffffLLHhB",	"ID,RawX,RawY,RX,RY,RZ,Dist,TSpan,DtSonar,FrmCnt,GT,Qlty"),
	LOG_FORMAT(GPOS, "LLfffffff",		"Lat,Lon,Alt,VelN,VelE,VelD,EPH,EPV,TALT"),
	LOG_FORMAT(GPSP, "LLffBfbf",		"Lat,Lon,Alt,Yaw,Type,LoitR,LoitDir,PitMin"),
	LOG_FORMAT(ESC, "HBBBHHffiffH",		"count,nESC,Conn,N,Ver,Adr,Volt,Amp,RPM,Temp,SetP,SetPRAW"),
	LOG_FORMAT(GVSP, "fff",			"VX,VY,VZ"),
	LOG_FORMAT(BATT, "fffffffB",		"V,VFilt,C,CFilt,Discharged,Remaining,Scale,Warning"),
	LOG_FORMAT(DIST, "BBBff",			"Id,Type,Orientation,Distance,Covariance"),
	LOG_FORMAT_S(TEL0, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT_S(TEL1, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT_S(TEL2, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT_S(TEL3, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT(EST0, "ffffffffffffBBHB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,nStat,fNaN,fFault,fTOut"),
	LOG_FORMAT(EST1, "ffffffffffffffff",	"s12,s13,s14,s15,s16,s17,s18,s19,s20,s21,s22,s23,s24,s25,s26,s27"),
	LOG_FORMAT(EST2, "ffffffffffffHHBH",     "P0,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,GCHK,CTRL,fHealth,IC"),
	LOG_FORMAT(EST3, "ffffffffffffffff",    "P12,P13,P14,P15,P16,P17,P18,P19,P20,P21,P22,P23,P24,P25,P26,P27"),
	LOG_FORMAT(EST4, "fffffffffffffff",     "VxI,VyI,VzI,PxI,PyI,PzI,VxIV,VyIV,VzIV,PxIV,PyIV,PzIV,e1,e2,e3"),
	LOG_FORMAT(EST5, "ffffffffffff", "MaxI,MayI,MazI,MaxIV,MayIV,MazIV,HeI,HeIV,AiI,AiIV,BeI,BeIV"),
	LOG_FORMAT(EST6, "ffffff", "FxI,FyI,FxIV,FyIV,HAGLI,HAGLIV"),
	LOG_FORMAT(PWR, "fffBBBBB",		"Periph5V,Servo5V,RSSI,UsbOk,BrickOk,ServoOk,PeriphOC,HipwrOC"),
	LOG_FORMAT(MOCP, "fffffff",		"QuatW,QuatX,QuatY,QuatZ,X,Y,Z"),
	LOG_FORMAT(VISN, "ffffffffff",		"X,Y,Z,VX,VY,VZ,QuatW,QuatX,QuatY,QuatZ"),
	LOG_FORMAT(GS0A, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(GS0B, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(GS1A, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(GS1B, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(TECS, "ffffffffffffffB",	"ASP,AF,FSP,F,AsSP,AsF,AsDSP,AsD,EE,ERE,EDE,EDRE,PtchI,ThrI,M"),
	LOG_FORMAT(WIND, "ffff",	"X,Y,CovX,CovY"),
	LOG_FORMAT(ENCD, "qfqf",	"cnt0,vel0,cnt1,vel1"),
	LOG_FORMAT(TSYN, "Q", 		"TimeOffset"),
	LOG_FORMAT(MACS, "fff", "RRint,PRint,YRint"),
	LOG_FORMAT(CAMT, "QI", "timestamp,seq"),
	LOG_FORMAT(RPL1, "QffQQffffffffff", "t,gIdt,aIdt,Tm,Tb,gx,gy,gz,ax,ay,az,magX,magY,magZ,b_alt"),
	LOG_FORMAT(RPL2, "QQLLiMMfffffffM", "Tpos,Tvel,lat,lon,alt,fix,nsats,eph,epv,sacc,v,vN,vE,vD,v_val"),
	LOG_FORMAT(RPL3, "QffffIB", "Tflow,fx,fy,gx,gy,delT,qual"),
	LOG_FORMAT(RPL4, "Qf", "Trng,rng"),
	LOG_FORMAT(RPL5, "Qfffffffff", "Tev,x,y,z,q0,q1,q2,q3,posErr,angErr"),
	LOG_FORMAT(RPL6, "Qff", "Tasp,inAsp,trAsp"),
	LOG_FORMAT(LAND, "B", "Landed"),
	LOG_FORMAT(LOAD, "f", "CPU"),
	LOG_FORMAT(DPRS, "Qffff", "errors,DPRESraw,DPRES,DPRESmax,Temp"),
	LOG_FORMAT(STCK, "NH", "Task,Free"),
	/* system-level messages, ID >= 0x80 */
	/* FMT: don't write format of format message, it's useless */
	LOG_FORMAT(TIME, "Q", "StartTime"),
	LOG_FORMAT(VER, "NZ", "Arch,FwGit"),
	LOG_FORMAT(PARM, "Zf", "Name,Value")
};

static const unsigned log_formats_num = sizeof(log_formats) / sizeof(log_formats[0]);

#endif

#endif /* SDLOG2_MESSAGES_H_ */
