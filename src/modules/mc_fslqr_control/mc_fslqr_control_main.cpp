/**
 * @file mc_fslqr_control_main.cpp
 * Multicopter frequency-shaped lqr controller.
 *
 * @author Kandai Watanabe <github account: kandai-wata>
 */

#include <iostream>
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
#include <px4_eigen.h>
#include <vector>
#include <mathlib/mathlib.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/param/param.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define MAX_GYRO_COUNT 3
#define MYPI 3.14159265358979323846f
#define TO_DEG 180f/MYPI
#define GRAVITY 9.8066f
#define GAZEBO_SOLO 1

/*
#define I_X 0.0067211875
#define I_Y 0.0080406875
#define I_Z 0.014278875
#define MASS 1.00
#define FMAX 4.179446268
#define ARM 0.2275
#define TMAX ARM*FMAX
#define DMAX 0.055562
*/
#define I_X 0.01147397f
#define I_Y 0.0154868f
#define I_Z 0.02185678f
#define MASS 1.5050f
#define MOT_X 0.14745
#define MOT_Y 0.14525
#define MOTOR_CONST 0.00000854858f
#define MOMENT_CONST 0.06
#define MOTOR_SCALE 1200
#define MOTOR_ARM 100
#define FMAX 14.4471002f
#define TXMAX 2.0984413f
#define TYMAX 2.13022492f
#define DMAX 0.866826012000000f
#define N_X 12
#define N_K 10


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
	enum {Xk1=0, Xk2, Xk3, Xk4, Xk5, Xk6, Xk7, Xk8, Xk9, Xk10, n_k};

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
	math::Matrix<3, 3> _Rot;			/**< rotation matrix from attitude quaternions */
	matrix::Dcmf _R_setpoint;
	math::Matrix<3, 3>	_board_rotation = {};	/**< rotation matrix for the orientation that the board is mounted */
	/*
	math::Matrix<n_u, n_x> K;
	math::Matrix<n_x, n_x> A;
	math::Matrix<n_x, n_u> B;
	math::Matrix<n_x, n_x> Q;
	math::Matrix<n_u, n_u> R;
	*/
	math::Matrix<n_u, n_x> _K;
	Eigen::MatrixXd _Keigen;
	Eigen::MatrixXd _A;
	Eigen::MatrixXd _B;
	Eigen::MatrixXd _Q;
	Eigen::MatrixXd _R;
  math::Vector<n_u> _U0;
	math::Matrix<n_u,n_u> _RealU2RealF;
	math::Matrix<n_u,n_u> _mixer;
	math::Matrix<n_u,n_u> _invNormMixer;
	math::Matrix<n_k,n_k> _Ak;
	math::Matrix<n_k,n_x> _Bk;
	math::Matrix<n_u,n_k> _Ck;
	math::Matrix<n_u,n_x> _Dk;
	math::Vector<n_k> _Xk;
	/*
	math::Matrix<4,4> _Const;
	math::Matrix<4,4> _mixer;
	math::Matrix<4,4> _normalize;
	math::Matrix<4,4> _U2PWM;
	*/

	void initAB();
	void setQR();
	void calcGainK();
	Eigen::MatrixXd care(const Eigen::MatrixXd A,
												const Eigen::MatrixXd B,
												const Eigen::MatrixXd Q,
												const Eigen::MatrixXd R);
	void calcFreqK();

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

	// math::Vector<n_k> rungekutta(math::Vector<n_k> Xk, math::Vector<n_x> X, math::Matrix<n_k,n_k> A, math::Matrix<n_k,n_x> B);
	math::Vector<N_K> rungekutta(math::Vector<N_K> Xk, math::Vector<N_X> X, math::Matrix<N_K,N_K> A, math::Matrix<N_K,N_X> B);

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
	_Rot.identity();
	_R_setpoint.identity();
	_board_rotation.identity();
	_actuators_id = ORB_ID(actuator_controls_0);
	parameters_update(true);

	/* rotation offsets */
	_params_handles.board_offset[0]		=	param_find("SENS_BOARD_X_OFF");
	_params_handles.board_offset[1]		=	param_find("SENS_BOARD_Y_OFF");
	_params_handles.board_offset[2]		=	param_find("SENS_BOARD_Z_OFF");

	// A.zero();
	// B.zero();
	// Q.zero();
	// R.zero();
	_K.zero();
	initAB();
	setQR();
	calcGainK();

	_RealU2RealF.zero();
	_mixer.zero();
	_invNormMixer.zero();
	/*
	float a = sqrt(2)/2;
	float temp[n_u][n_u] = {{-a, a, 1, 1},
													{a, -a, 1, 1},
													{a, a, -1, 1},
													{-a, -a, -1, 1}};
	math::Matrix<n_u,n_u> _U2F_norm(temp);
	*/
	float pmet[n_u][n_u]={{-0.25,	0.25,		0.25,		0.25},
												{0.25,	-0.25,	0.25,		0.25},
												{0.25,	0.25,		-0.25,	0.25},
												{-0.25,	-0.25,	-0.25,	0.25}};
	math::Matrix<n_u,n_u> tempo(pmet);
	_mixer = tempo;
	float b = sqrt(2)/4;
	float c = 0.25;
	float abcd[n_u][n_u]={{-b,	b,	b,	-b},
												{b, 	-b,	b,	-b},
												{c, 	c,	-c,	-c},
												{c, 	c,	c,	c}};
	math::Matrix<n_u,n_u> tempor(abcd);
	_invNormMixer = tempor;
	math::Matrix<n_u,n_u> Const;
	Const.zero();
	Const(0,0) = MOT_Y;
	Const(1,1) = MOT_X;
	Const(2,2) = MOMENT_CONST;
	Const(3,3) = 1;
	_RealU2RealF = _mixer * Const.inversed();

	_Ak.zero();
	_Bk.zero();
	_Ck.zero();
	_Dk.zero();
	_Xk.zero();
	calcFreqK();

  _U0.zero();
	_U0(Uf) = MASS*GRAVITY;
}

void
MulticopterFSLQRControl::initAB()
{
	_A = Eigen::MatrixXd::Zero(n_x, n_x);
	_B = Eigen::MatrixXd::Zero(n_x, n_u);

	_A(Xx, Xu) = 1;
	_A(Xy, Xv) = 1;
	_A(Xz, Xw) = 1;
	_A(Xu, Xtheta) = -CONSTANTS_ONE_G;
	_A(Xv, Xphi) = CONSTANTS_ONE_G;
	_A(Xphi, Xp) = 1;
	_A(Xtheta, Xq) = 1;
	_A(Xpsi, Xr) = 1;

	_B(Xw, Uf) = -1/MASS;
	_B(Xp, Utx) = 1/I_X;
	_B(Xq, Uty) = 1/I_Y;
	_B(Xr, Utz) = 1/I_Z;
	/*
	std::cout << "_A" << std::endl;
	std::cout << _A << std::endl;
	std::cout << "_B" << std::endl;
	std::cout << _B << std::endl;
	*/
}

void
MulticopterFSLQRControl::setQR()
{
	_Q = Eigen::MatrixXd::Identity(n_x, n_x);
	_R = Eigen::MatrixXd::Identity(n_u, n_u);

	bool isPosition=true;
	if(isPosition){
		_Q(Xx, Xx) = 10;
		_Q(Xy, Xy) = 10;
		_Q(Xz, Xz) = 10;
	}else{
		_Q(Xphi, Xphi) = 100;
		_Q(Xtheta, Xtheta) = 100;
		_Q(Xpsi, Xpsi) = 0.1;
	}
	/*
	std::cout << "_Q" << std::endl;
	std::cout << _Q << std::endl;
	std::cout << "_R" << std::endl;
	std::cout << _R << std::endl;
	*/
}


void
MulticopterFSLQRControl::calcGainK()
{
	/**
	* Calculate LQR Gain K
	* Solves Riccati Equation using Arimoto Potter Method
	*/
	Eigen::MatrixXd  P = care(_A, _B, _Q, _R);
	_Keigen = _R.inverse() * _B.transpose() * P;

	for(int i(0); i<n_u; i++){
		for(int j(0); j<n_x; j++){
			_K(i,j) = float(_Keigen(i,j));
		}
	}
}

void
MulticopterFSLQRControl::calcFreqK()
{
	Eigen::MatrixXd Ak(10,10);
	Eigen::MatrixXd Bk(10,12);
	Eigen::MatrixXd Ck(4,10);
	Eigen::MatrixXd Dk(4,12);

	Ak << 0, -3.155, 0, 0, 0, 0, 0, 0, 0, 0,
     3.1282, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, -3.155, 0, 0, 0, 0, 0, 0,
     0, 0, 3.1282, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, -3.155, 0, 0, 0, 0,
     0, 0, 0, 0, 3.1282, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, -0.062832, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, -0.062832, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, -0.062832, 0,
     -5.6393e-14, 7.9706e-14, 6.1479e-15, -3.8897e-15, 1.8931e-14, -1.3935e-14, 1.9106e-13, -5.5343e-14, 5.3764e-15, -99.7813;
	Bk << 5.5975, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 5.5975, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 5.5975, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 7.9227, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 7.9227, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 7.9227, 0, 0, 0, 0, 0, 0,
     -1.8074e-12, 3.0629e-13, 2.3377e-14, -8.7886e-14, -4.4975e-14, -7.8793e-16, -6.5393e-14, 1.258e-13, -7.9267, -8.3926e-16, 1.9157e-15, -8.1972;
	Ck << -1.9179e-15, 7.2614e-16, -1.1366e-15, -1.9315e-15, -0.38413, -1.3637, -9.6118e-15, 1.3669e-14, 0.74321, 8.3324e-18,
     1.0904e-14, -5.6964e-15, 0.027255, 1.417, -6.4998e-15, 1.6655e-14, 1.5505e-13, -0.74737, -1.4281e-14, -1.1641e-15,
     -0.044655, -1.4165, -1.4532e-16, 3.9808e-15, 4.8676e-15, -5.3552e-15, 0.74719, -8.9402e-14, 6.044e-15, 1.9687e-15,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 7.9267;
	Dk << 7.2337e-14, -1.2595e-13, 4.456, -8.5038e-16, -6.4812e-15, 5.6689, -9.8929e-15, 1.9931e-15, -2.2651e-16, -7.6134e-17, -8.3265e-18, -1.7304e-18,
     -1.2613e-12, -4.5505, 8.5721e-14, -1.09e-14, -4.7596, 9.9862e-15, -10.1142, 5.9786e-15, -2.3111e-15, -1.11, 2.0584e-17, -2.9287e-16,
     4.5497, 7.1583e-13, -2.1325e-14, 4.8, -3.9871e-15, 8.0917e-16, -9.6432e-15, -10.2943, 7.5299e-16, 1.525e-17, -1.1484, 4.3265e-16,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

	for(int i(0); i<n_k; i++){
		for(int j(0); j<n_k; j++){
			_Ak(i,j) = float(Ak(i,j));
		}
	}
	for(int i(0); i<n_k; i++){
		for(int j(0); j<n_x; j++){
			_Bk(i,j) = float(Bk(i,j));
		}
	}
	for(int i(0); i<n_u; i++){
		for(int j(0); j<n_k; j++){
			_Ck(i,j) = float(Ck(i,j));
		}
	}
	for(int i(0); i<n_u; i++){
		for(int j(0); j<n_x; j++){
			_Dk(i,j) = float(Dk(i,j));
		}
	}
}

Eigen::MatrixXd
MulticopterFSLQRControl::care(const Eigen::MatrixXd A,
	 														const Eigen::MatrixXd B,
															const Eigen::MatrixXd Q,
															const Eigen::MatrixXd R)
{
		// Continuous Algebraic Riccati Equation Solver
		// http://www.humachine.fr.dendai.ac.jp/lec/SDRE_%E3%83%86%E3%82%AF%E3%83%8E%E3%82%BB%E3%83%B3%E3%82%BF%E8%AC%9B%E6%BC%9405.pdf
    int n = (int)A.rows();
		// Hamilton Matrix
    Eigen::MatrixXd Ham(2*n, 2*n);
    Ham << A, -B*R.inverse()*B.transpose(), -Q, -A.transpose();

    // EigenVec, Value
    Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);
    if (Eigs.info() != Eigen::Success) abort();

		Eigen::MatrixXcd eigvec(2*n, n);
    int j = 0;

    for(int i = 0; i < 2*n; ++i){
        if(Eigs.eigenvalues()[i].real() < 0){
            eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2*n, 1);
            ++j;
        }
    }

		Eigen::MatrixXcd U(n, n);
		Eigen::MatrixXcd V(n, n);

		U = eigvec.block(0,0,n,n);
    V = eigvec.block(n,0,n,n);
    Eigen::MatrixXd  P = (V * U.inverse()).real();

    //解Pを求める
    return P;
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
		_Rot = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _Rot.to_euler();
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
  /*
    Motor placement: @ Body Coordinate
    		 ^ x
    (2)  |   (0)
    		 |
    -----------> y
    		 |
    (1)  |   (3)
    		 |

		px4 Coordinate (World Coordinate)
    		 ^ y
    (2)  |   (0)
    		 |
 x <------------
    		 |
    (1)  |   (3)
  */
	// Rotation is around body coordinate. Except for yaw which needs reference frame in  world coordinate
	math::Vector<n_x> X;
	//X(Xx) = _local_posy - _pos_sp_triplet.current.y;
	//X(Xy) = -(_local_pos.x - _pos_sp_triplet.current.x);
	// X(Xz) = _local_pos.z - _pos_sp_triplet.current.z;
	X(Xx) = _local_pos.y;
	X(Xy) = -_local_pos.x;
	X(Xz) = _local_pos.z + 1;
	X(Xu) = _local_pos.vy;
	X(Xv) = -_local_pos.vx;
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
	X(Xpsi) = eul(2)-MYPI/2;
	X(Xp) = PX4_ISFINITE(rates(0)) ? rates(0) : X(Xp);
	X(Xq) = PX4_ISFINITE(rates(1)) ? rates(1) : X(Xq);
	X(Xr) = PX4_ISFINITE(rates(2)) ? rates(2) : X(Xr);

	// std::cout << _Keigen << std::endl;
	math::Vector<n_u> U;
	math::Vector<n_u> KX;

	//if(_local_pos.z < -1.0f){
	if(false){
		std::cout << "Now in Frequency-Shaped LQR "  << std::endl;
		float dt = getDt();
		math::Vector<n_k> dXk1;
		math::Vector<n_k> dXk2;
		math::Vector<n_k> dXk3;
		math::Vector<n_k> dXk4;
		dXk1 = rungekutta(_Xk, X, _Ak, _Bk)*dt;
		dXk2 = rungekutta(_Xk+dXk1/2, X, _Ak, _Bk)*dt;
		dXk3 = rungekutta(_Xk+dXk2/2, X, _Ak, _Bk)*dt;
		dXk4 = rungekutta(_Xk+dXk3, X, _Ak, _Bk)*dt;
		_Xk = _Xk+(dXk1+dXk2*2.0f+dXk3*2.0f+dXk4)/6.0f;
		U = _Ck*_Xk + _Dk*X + _U0;
	}else{
		U = _U0 - _K*X;
		KX = -_K*X;
	}

	/* BE CAREFUL U is in order of Uf, Utx, Uty, Utz */
	// trying to calculate required normalized U from Motor Dynamics expressed in Gazebo
	// please check Firmware/Tools/sitl_gazebo/src/gazebo_motor_model.cpp
	math::Vector<n_u> orderedU({ U(Utx), U(Uty), U(Utz), U(Uf) });
	math::Vector<n_u> Fdes = _RealU2RealF*orderedU;
	// std::cout << "Fdes: "  << Fdes(0) << " " << Fdes(1) << " " << Fdes(2) << " " << Fdes(3) << std::endl; << " " << KX(Uty) << " " << KX(Utz) << "  " << KX(Uf) << std::endl;

	bool ue4=true;
	math::Vector<n_u> Wref;
	math::Vector<n_u> f_fact;
	if(ue4){
		for(int i=0; i<n_u; i++){
			f_fact(i) = ((Fdes(i)/FMAX) - 0.2f)/0.8f;
		}
	}else{
		Fdes /= MOTOR_CONST;
		for(int i=0; i<n_u; i++){
			if(Fdes(i)<0){
				Wref(i)=sqrt(-Fdes(i));
				f_fact(i) = -((Wref(i)-MOTOR_ARM)/MOTOR_SCALE)*((Wref(i)-MOTOR_ARM)/MOTOR_SCALE);
			}else{
				Wref(i) = sqrt(Fdes(i));
				f_fact(i) = ((Wref(i)-MOTOR_ARM)/MOTOR_SCALE)*((Wref(i)-MOTOR_ARM)/MOTOR_SCALE);
			}
		}
	}
	math::Vector<n_u> normU = _invNormMixer * f_fact;
	// std::cout << "Wref: " << Wref(0) << " " << Wref(1) << " " << Wref(2) << " " << Wref(3) << std::endl;
	// std::cout << "f_fact: " << f_fact(0) << " " << f_fact(1) << " " << f_fact(2) << " " << f_fact(3) << std::endl;

	bool manualInput=false;
	if(manualInput){
		_actuators.control[0] = 0.0f;
		_actuators.control[1] = 0.0f;
		_actuators.control[2] = 0.0f;
		_actuators.control[3] = 1.0f;	 // + m*g for initial U0
	}else{
		_actuators.control[0] = PX4_ISFINITE(normU(0)) ? normU(0) : 0.0f;
		_actuators.control[1] = PX4_ISFINITE(normU(1)) ? normU(1) : 0.0f;
		_actuators.control[2] = PX4_ISFINITE(normU(2)) ? normU(2) : 0.0f;
		_actuators.control[3] = PX4_ISFINITE(normU(3)) ? normU(3) : 0.0f;
	}

	_actuators.timestamp = hrt_absolute_time();
	_actuators.timestamp_sample = _sensor_gyro.timestamp;

  float diff = (hrt_absolute_time()-_last_time) / 1000000.0f;
  if(diff > 0.5f){
		std::cout << "gyro: " << _sensor_gyro.x << " " << _sensor_gyro.y << " " << _sensor_gyro.z << std::endl;
		std::cout << "xyz: " << X(Xx) << " " << X(Xy) << " " <<  X(Xz) << std::endl;
		std::cout << "uvw: " << X(Xu) << " " << X(Xv) << " " <<  X(Xw) << std::endl;
		std::cout << "eul: " << X(Xphi) << " " << X(Xtheta) << " " <<  X(Xpsi) << std::endl;
		std::cout << "pqr: " << X(Xp) << " " << X(Xq) << " " <<  X(Xr) << std::endl;
		/*
		std::cout << "ref pos: " << _pos_sp_triplet.current.y << " " << -_pos_sp_triplet.current.x << " " << _pos_sp_triplet.current.z << std::endl;
		std::cout << "KX: " << KX(Utx) << " " << KX(Uty) << " " << KX(Utz) << "  " << KX(Uf) << std::endl;
		std::cout << "U: " << U(Utx) << " " << U(Uty) << " " << U(Utz) << " " << U(Uf) << std::endl;
		std::cout << "f_fact: " << f_fact(0) << " " << f_fact(1) << " " << f_fact(2) << " " << f_fact(3) << std::endl;
		*/
  	_last_time=hrt_absolute_time();
  }
}

math::Vector<N_K>
MulticopterFSLQRControl::rungekutta(math::Vector<N_K> Xk, math::Vector<N_X> X, math::Matrix<N_K,N_K> A, math::Matrix<N_K,N_X> B)
{
	return A*Xk + B*X;
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

		/* guard against too small (< 2ms) and too large (> 20ms) dt's */
		if (dt < 0.002f) {
			dt = 0.002f;

		} else if (dt > 0.02f) {
			dt = 0.02f;
		}


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
