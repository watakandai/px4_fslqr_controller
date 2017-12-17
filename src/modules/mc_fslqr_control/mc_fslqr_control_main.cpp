/**
 * @file mc_fslqr_control_main.cpp
 * Multicopter frequency-shaped lqr controller.
 *
 * @author Kandai Watanabe <github account: kandai-wata>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <conversion/rotation.h>
#include <float.h>
#include <lib/geo/geo.h>
#include <lib/mixer/mixer.h>
// #include <lib/eigen/Eigen/Core>
#include <px4_eigen.h>
#include <mathlib/mathlib.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/param/param.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define MAX_GYRO_COUNT 3
#define MYPI 3.14159265358979323846f
#define TO_DEG 180/MYPI
#define I_X 0.0347563
#define I_Y 0.0458929
#define I_Z 0.0977
#define MAX_VEL 1100
#define MOTOR_CONST = 0.00000

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_fslqr_control_main(int argc, char *argv[]);

class MulticopterFSLQRControl : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	MulticopterFSLQRControl();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterFSLQRControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool		_task_should_exit;			/**<true if task should exit */
	int		_control_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_vehicle_attitude_sub;		/**< control state subscription */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int 	_motor_limits_sub;
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_home_pos_sub; 			/**< home position */

	int		_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
	int		_sensor_correction_sub;	/**< sensor thermal correction subscription */
	int		_sensor_bias_sub;	/**< sensor in-run bias correction subscription */

	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */
	orb_advert_t 	_local_pos_sp_pub;

	orb_id_t _attitude_setpoint_id;
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct mc_att_ctrl_status_s 		_mc_att_ctrl_status;
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct home_position_s				_home_pos; 				/**< home position */

	struct sensor_gyro_s			_sensor_gyro;		/**< gyro data before thermal correctons and ekf bias estimates are applied */
	struct sensor_correction_s		_sensor_correction;	/**< sensor thermal corrections */
	struct sensor_bias_s			_sensor_bias;		/**< sensor in-run bias corrections */
	struct map_projection_reference_s _ref_pos;
	struct actuator_controls_s		_actuators;		/**< actuator controls */
	MultirotorMixer::saturation_status _saturation_status{};

	struct {
		param_t board_rotation;
		param_t board_offset[3];
	} _params_handles;

	struct {
		int32_t board_rotation;
		float board_offset[3];
	} _params;

	enum {Xx=0, Xy, Xz, Xu, Xv, Xw, Xphi, Xtheta, Xpsi, Xp, Xq, Xr, n_x};
	enum {Uf=0, Utx, Uty, Utz, n_u};

	float _ref_alt;
	hrt_abstime _ref_timestamp;
	unsigned _gyro_count;
	int _selected_gyro;
	float _yaw;				/**< yaw angle (euler) */
	uint8_t _sign;
	uint64_t _last_time;
	int _arm_length;

	uint8_t _z_reset_counter;
	uint8_t _xy_reset_counter;
	uint8_t _heading_reset_counter;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	matrix::Dcmf _R_setpoint;
	math::Matrix<3, 3>	_board_rotation = {};	/**< rotation matrix for the orientation that the board is mounted */
	math::Matrix<n_u, n_x> K;
  math::Vector<n_u> U0;



	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();

	void 		do_control();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace fslqr_control
{
	MulticopterFSLQRControl	*g_control;
}


MulticopterFSLQRControl::MulticopterFSLQRControl() :
	SuperBlock(nullptr, "MPC"),
	_task_should_exit(false),
	_control_task(-1),
	_mavlink_log_pub(nullptr),
	/* subscriptions */
	_vehicle_status_sub(-1),
	_vehicle_land_detected_sub(-1),
	_vehicle_attitude_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_motor_limits_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_home_pos_sub(-1),
	_sensor_correction_sub(-1),
	_sensor_bias_sub(-1),
	/* publications */
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_attitude_setpoint_id(nullptr),
	_actuators_0_circuit_breaker_enabled(false),
	_vehicle_status{},
	_mc_att_ctrl_status{},
	_vehicle_land_detected{},
	_att{},
	_manual{},
	_control_mode{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_home_pos{},
	_sensor_gyro{},
	_sensor_correction{},
	_sensor_bias{},
	_ref_pos{},
	_actuators{},
	_saturation_status{},

	_ref_alt(0.0f),
	_ref_timestamp(0),
	_gyro_count(1),
	_selected_gyro(0),
	_yaw(0.0f),
	_sign(0),
	_last_time(0),
	_arm_length(0.23),

	_z_reset_counter(0),
	_xy_reset_counter(0),
	_heading_reset_counter(0)
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}

	_params.board_rotation = 0;
	_params.board_offset[0] = 0.0f;
	_params.board_offset[1] = 0.0f;
	_params.board_offset[2] = 0.0f;
	_params_handles.board_rotation		=	param_find("SENS_BOARD_ROT");

	_pos.zero();
	_pos_sp.zero();
	_R.identity();
	_R_setpoint.identity();
	_board_rotation.identity();
	_actuators_id = ORB_ID(actuator_controls_0);
	parameters_update(true);

	/* rotation offsets */
	_params_handles.board_offset[0]		=	param_find("SENS_BOARD_X_OFF");
	_params_handles.board_offset[1]		=	param_find("SENS_BOARD_Y_OFF");
	_params_handles.board_offset[2]		=	param_find("SENS_BOARD_Z_OFF");

	/*
	K(Utx, Xx) = -9.5560*10^(-16);
	K(Utx, Xy) = -9.5110*10^(-16);
	K(Utx, Xz) = 3.0984;
	K(Utx, Xu) = 1.5537*10^(-17);
	K(Utx, Xv) = -1.0964*10^(-16);
	K(Utx, Xw) = 4.2999;
	K(Utx, Xphi) = 9.6986*10^(-16);
	K(Utx, Xtheta) = -1.0722*10^(-15);
	K(Utx, Xpsi) = -1.7784*10^(-16);
	K(Utx, Xp) = -7.2540*10^(-18);
	K(Utx, Xq) = -8.5048*10^(-18);
	K(Utx, Xr) = -1.821*10^(-17);

	K(Uty, Xx) = 1.8619*10^(-14);
	K(Uty, Xy) = -2.9828;
	K(Uty, Xz) = 2.4418*10^(-16);
	K(Uty, Xu) = 3.1208*10^(-15);
	K(Uty, Xv) = -3.9486;
	K(Uty, Xw) = -4.5379*10^(-16);
	K(Uty, Xphi) = -9.5939;
	K(Uty, Xtheta) = 1.2636*10^(-15);
	K(Uty, Xpsi) = -5.0583*10^(-15);
	K(Uty, Xp) = -1.1917;
	K(Uty, Xq) = -2.6702*10^(-17);
	K(Uty, Xr) = -3.0129*10^(-16);

	K(Utz, Xx) = 2.9725;
	K(Utz, Xy) = -8.7596*10^(-15);
	K(Utz, Xz) = -5.5850*10^(-16);
	K(Utz, Xu) = 3.9179*10^(-15);
	K(Utz, Xv) = 3.9486;
	K(Utz, Xw) = -1.069*10^(-15);
	K(Utz, Xphi) = 2.7745*10^(-15);
	K(Utz, Xtheta) = -9.1972;
	K(Utz, Xpsi) = 2.0776*10^(-15);
	K(Utz, Xp) = -5.3649*10^(-17);
	K(Utz, Xq) = -1.0957;
	K(Utz, Xr) = 5.9262*10^(-18);

	K(Uf, Xx) = 9.9189*10^(-15);
	K(Uf, Xy) = -1.7924*10^(-14);
	K(Uf, Xz) = 1.9088*10^(-15);
	K(Uf, Xu) = -8.5873*10^(-16);
	K(Uf, Xv) = -5.3144*10^(-15);
	K(Uf, Xw) = 8.3818*10^(-16);
	K(Uf, Xphi) = -1.2725*10^(-14);
	K(Uf, Xtheta) = -5.4829*10^(-16);
	K(Uf, Xpsi) = -1.7621;
	K(Uf, Xp) = -2.1563*10^(-16);
	K(Uf, Xq) = 2.111*10^(-18);
	K(Uf, Xr) = -1.0525;
	*/
	K.zero();
	K(Uf, Xz) = 3.0984;
	K(Uf, Xw) = 4.2999;

	K(Utx, Xy) = -2.9812;
	K(Utx, Xv) = -3.9434;
	K(Utx, Xphi) = -9.5264;
	K(Utx, Xp) = -1.1752;

	K(Uty, Xx) = 2.9811;
	K(Uty, Xu) = 3.9434;
	K(Uty, Xtheta) = -9.5264;
	K(Uty, Xq) = -1.1752;

	K(Utz, Xpsi) = -1.7615;
	K(Utz, Xr) = -1.0515;

  U0.zero();
  U0(Uf) = 9.81*1.30;
}

MulticopterFSLQRControl::~MulticopterFSLQRControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	fslqr_control::g_control = nullptr;
}

int
MulticopterFSLQRControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;
	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		/* update C++ param system */
		updateParams();
		_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);
		/* rotation of the autopilot relative to the body */
		param_get(_params_handles.board_rotation, &(_params.board_rotation));

		/* fine adjustment of the rotation */
		param_get(_params_handles.board_offset[0], &(_params.board_offset[0]));
		param_get(_params_handles.board_offset[1], &(_params.board_offset[1]));
		param_get(_params_handles.board_offset[2], &(_params.board_offset[2]));

		/* get transformation matrix from sensor/board to body frame */
		get_rot_matrix((enum Rotation)_params.board_rotation, &_board_rotation);

		/* fine tune the rotation */
		math::Matrix<3, 3> board_rotation_offset;
		board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _params.board_offset[0],
			M_DEG_TO_RAD_F * _params.board_offset[1],
			M_DEG_TO_RAD_F * _params.board_offset[2]);
		_board_rotation = board_rotation_offset * _board_rotation;
	}

	return OK;
}

void
MulticopterFSLQRControl::poll_subscriptions()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_attitude_setpoint_id) {
			_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
		}
	}

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	orb_check(_vehicle_attitude_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);

		/* get current rotation matrix and euler angles from control state quaternions */
		math::Quaternion q_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
		_R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _R.to_euler();
		_yaw = euler_angles(2);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

		// check if a reset event has happened
		// if the vehicle is in manual mode we will shift the setpoints of the
		// states which were reset. In auto mode we do not shift the setpoints
		// since we want the vehicle to track the original state.
		if (_control_mode.flag_control_manual_enabled) {
			if (_z_reset_counter != _local_pos.z_reset_counter) {
				_pos_sp(2) = _local_pos.z;
			}

			if (_xy_reset_counter != _local_pos.xy_reset_counter) {
				_pos_sp(0) = _local_pos.x;
				_pos_sp(1) = _local_pos.y;
			}
		}

		// update the reset counters in any case
		_z_reset_counter = _local_pos.z_reset_counter;
		_xy_reset_counter = _local_pos.xy_reset_counter;
	}

	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		/* to be a valid current triplet, altitude has to be finite */

		if (!PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
		}

		/* to be a valid previous triplet, lat/lon/alt has to be finite */

		if (!PX4_ISFINITE(_pos_sp_triplet.previous.lat) ||
		    !PX4_ISFINITE(_pos_sp_triplet.previous.lon) ||
		    !PX4_ISFINITE(_pos_sp_triplet.previous.alt)) {
			_pos_sp_triplet.previous.valid = false;
		}
	}

	orb_check(_home_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}

	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		multirotor_motor_limits_s motor_limits = {};
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &motor_limits);

		_saturation_status.value = motor_limits.saturation_status;
	}

	orb_check(_sensor_correction_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
	}

	/* update the latest gyro selection */
	if (_sensor_correction.selected_gyro_instance < _gyro_count) {
		_selected_gyro = _sensor_correction.selected_gyro_instance;
	}

	orb_check(_sensor_bias_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_bias), _sensor_bias_sub, &_sensor_bias);
	}

}

void
MulticopterFSLQRControl::update_ref()
{
	// The reference point is only allowed to change when the vehicle is in standby state which is the
	// normal state when the estimator origin is set. Changing reference point in flight causes large controller
	// setpoint changes. Changing reference point in other arming states is untested and shoud not be performed.
	if ((_local_pos.ref_timestamp != _ref_timestamp)
	    && (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY)) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			// calculate current position setpoint in global frame
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			// the altitude setpoint is the reference altitude (Z up) plus the (Z down)
			// NED setpoint, multiplied out to minus
			alt_sp = _ref_alt - _pos_sp(2);
		}

		// update local projection reference including altitude
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_ref_timestamp != 0) {
			// reproject position setpoint to new reference
			// this effectively adjusts the position setpoint to keep the vehicle
			// in its current local position. It would only change its
			// global position on the next setpoint update.
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

void
MulticopterFSLQRControl::do_control()
{
	math::Vector<n_x> X;
	X(Xx) = _local_pos.x - _pos_sp(Xx);
	X(Xy) = _local_pos.y - _pos_sp(Xy);
	X(Xz) = _local_pos.z - _pos_sp(Xz);
	X(Xu) = _local_pos.vx;
	X(Xv) = _local_pos.vy;
	X(Xw) = _local_pos.vz;

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
	math::Vector<3> eul = q_att.to_euler();

	math::Vector<3> rates;
	if (_selected_gyro == 0) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		rates(0) = _sensor_gyro.x;
		rates(1) = _sensor_gyro.y;
		rates(2) = _sensor_gyro.z;
	}
	// rotate corrected measurements from sensor to body frame
	rates = _board_rotation * rates;
	// correct for in-run bias errors
	rates(0) -= _sensor_bias.gyro_x_bias;
	rates(1) -= _sensor_bias.gyro_y_bias;
	rates(2) -= _sensor_bias.gyro_z_bias;

	X(Xphi) = eul(0);
	X(Xtheta) = eul(1);
	X(Xpsi) = eul(2);
	X(Xp) = rates(0);
	X(Xq) = rates(1);
	X(Xr) = rates(2);
	math::Vector<n_u> U;
	U = U0 + K*X;
	U(Uf) = U(Uf)/(2*U0(Uf));

	_actuators.control[0] = U(Utx);
	_actuators.control[1] = U(Uty);
	_actuators.control[2] = U(Utz);
	_actuators.control[3] = U(Uf); // + m*g for initial U0
	_actuators.timestamp = hrt_absolute_time();

  float diff = (hrt_absolute_time()-_last_time) / 1000000.0f;
  if(diff > 1.0f){
		mavlink_log_critical(&_mavlink_log_pub, "x_sp: %3.3f", double(_pos_sp(Xx)));
		mavlink_log_critical(&_mavlink_log_pub, "y_sp: %3.3f", double(_pos_sp(Xy)));
		mavlink_log_critical(&_mavlink_log_pub, "z_sp: %3.3f", double(_pos_sp(Xz)));
		mavlink_log_critical(&_mavlink_log_pub, "x: %3.3f", double(X(Xx)));
		mavlink_log_critical(&_mavlink_log_pub, "y: %3.3f", double(X(Xy)));
		mavlink_log_critical(&_mavlink_log_pub, "z: %3.3f", double(X(Xz)));
		mavlink_log_critical(&_mavlink_log_pub, "phi: %3.3f", double(X(Xphi)));
		mavlink_log_critical(&_mavlink_log_pub, "theta: %3.3f", double((Xtheta)));
		mavlink_log_critical(&_mavlink_log_pub, "psi: %3.3f", double(X(Xpsi)));
		mavlink_log_critical(&_mavlink_log_pub, "Utx: %3.3f", double(U(Utx)));
		mavlink_log_critical(&_mavlink_log_pub, "Uty: %3.3f", double(U(Uty)));
		mavlink_log_critical(&_mavlink_log_pub, "Utz: %3.3f", double(U(Utz)));
		mavlink_log_critical(&_mavlink_log_pub, "Uf: %3.3f", double(U(Uf)));
    if(_sign==0){
      _sign=1;
      // _actuators.control[3] = 0.65f;
    }else{
      _sign=0;
      // _actuators.control[3] = 0.5f;
    }
    _last_time=hrt_absolute_time();
  }
}

void
MulticopterFSLQRControl::task_main_trampoline(int argc, char *argv[])
{
	fslqr_control::g_control->task_main();
}

void
MulticopterFSLQRControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));

	_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);
	if (_gyro_count == 0) {
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));
	_sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));

	parameters_update(true);

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	hrt_abstime t_prev = 0;

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;
	fds[1].events = POLLIN;

	while (!_task_should_exit) {
		fds[1].fd = _sensor_gyro_sub[_selected_gyro];
		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			// Go through the loop anyway to copy manual input at 50 Hz.
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
		t_prev = t;

		/* set dt for control blocks */
		setDt(dt);

		/* copy gyro data */
		orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);
		poll_subscriptions();
		parameters_update(false);
		update_ref();

		/* Choose
		* 	1. use both fslqr, att, pos control
		* 			- if offboard -> fslqr
		* 			- else att,pos control
		* 	2. use fslqr only, delete att & pos control
		* 			- handle all auto, manual, offboard in fslqr
		*/
		if(_control_mode.flag_control_offboard_enabled){
			do_control();
		}

		if (!_actuators_0_circuit_breaker_enabled) {
			if (_actuators_0_pub != nullptr) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
			} else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}
		}
	}
	mavlink_log_info(&_mavlink_log_pub, "[mfslqrc] stopped");

	_control_task = -1;
}

int
MulticopterFSLQRControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_fslqr_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_POSITION_CONTROL,
					   1900,
					   (px4_main_t)&MulticopterFSLQRControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_fslqr_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_fslqr_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (fslqr_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		fslqr_control::g_control = new MulticopterFSLQRControl;

		if (fslqr_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != fslqr_control::g_control->start()) {
			delete fslqr_control::g_control;
			fslqr_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (fslqr_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete fslqr_control::g_control;
		fslqr_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (fslqr_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
