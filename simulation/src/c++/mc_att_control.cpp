/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Roman Bapst <bapstr@ethz.ch>
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>


#include <math/Vector.hpp>
#include <math/Matrix.hpp>
#include <math/Limits.hpp>
#include <math/Quaternion.hpp>
#include <geo/geo.h>

#include <ros/ros.h>
#include <time.h>

#include <mavros/ManualControl.h>
#include <mavros/AttitudeRates.h>
#include <mavros/ArmedState.h>

#include <simulation/mixer_out.h>
#include <simulation/simout.h>

#include <manual_control_setpoint.h>
#include <vehicle_attitude.h>
#include <vehicle_attitude_setpoint.h>
#include <vehicle_rates_setpoint.h>
#include <vehicle_control_mode.h>
#include <actuator_armed.h>
#include <actuator_controls.h>

#include "mc_mixer.cpp"

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f


class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();


	/**
	 * Destructor still to come
	 */
	//~MulticopterAttitudeControl();


private:
	//ros subscribers
	ros::NodeHandle n;
	ros::Subscriber attitude_sub;
	ros::Subscriber rc_sub;
	ros::Subscriber armed_status_sub;

	//ros publishers
	ros::Publisher _actuators_0_pub;
	ros::Publisher att_rates_pub;
	ros::Publisher mixer_out_pub;

	double time_last;	//used to measure elapsed time

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	//
	struct vehicle_attitude_s			_v_att;				/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_armed_s				_armed;				/**< actuator arming status */


	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Matrix<3, 3>  _I;				/**< identity matrix */

	MultirotorMixer *mcMixer;


//
	bool	_reset_yaw_sp;			/**< reset yaw setpoint flag */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		float yaw_ff;						/**< yaw control feed-forward */
		float yaw_rate_max;					/**< max yaw rate */

		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */
	}		_params;


	void attitude_cb(const simulation::simout msg);
	void rc_cb(const mavros::ManualControl msg);
	void armed_status_cb(mavros::ArmedState msg);

	void fill_rates_msg(mavros::AttitudeRates &msg);

	void task_main(); //run the main task, this needs to be triggerd by a callback, not sure yet which one
	void control_attitude(double dt);
	void control_attitude_rates(float dt);
	void mix_and_publish();		//mix desired controls and publish on topic (Simulator can subscribe to it)

	double get_dt();	//get time since last call [seconds]
};

MulticopterAttitudeControl::MulticopterAttitudeControl()
{
	//setup all subscribers
	attitude_sub = n.subscribe("simout",1000,&MulticopterAttitudeControl::attitude_cb,this);
	rc_sub = n.subscribe("mavros/ManualSetpoint",1000,&MulticopterAttitudeControl::rc_cb,this);
	armed_status_sub = n.subscribe("mavros/ArmedState",1000,&MulticopterAttitudeControl::armed_status_cb,this);

	//setup all publishers
	att_rates_pub = n.advertise<mavros::AttitudeRates>("MulticopterAttitudeControl/att_rates",10);
	mixer_out_pub = n.advertise<simulation::mixer_out>("mixer_out",10);


	//setup time variable, this is used by the class-method 'get_dt()'
	time_last = ros::Time::now().toSec();

	//init circuit-braker
	_actuators_0_circuit_breaker_enabled = false;

	//create Mixer
	mcMixer = new MultirotorMixer();

	//initialize topic data holders
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	//memset(&_armed, 0, sizeof(_armed));
	_armed.armed = true;

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_d.zero();
	_params.yaw_ff = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.man_roll_max = 0.0f;
	_params.man_pitch_max = 0.0f;
	_params.man_yaw_max = 0.0f;
	_params.acro_rate_max.zero();

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();

	//set control mode flags, this is a hack, need to get them from the board
	_v_control_mode.flag_control_attitude_enabled = true;
	_v_control_mode.flag_control_rates_enabled = true;
	_v_control_mode.flag_control_manual_enabled = true;
	_v_control_mode.flag_control_climb_rate_enabled = false;
	_v_control_mode.flag_control_velocity_enabled = false;
	_v_control_mode.flag_control_auto_enabled = true;
	_v_control_mode.flag_control_position_enabled = true;


	//set parameters, hard-coded for now
	/* roll gains */
	_params.att_p(0) = 10;
	_params.rate_p(0) = 0.2;
	_params.rate_i(0) = 0.01;
	_params.rate_d(0) = 0.05;

	/* pitch gains */
	_params.att_p(1) = 10.0;
	_params.rate_p(1) = 0.2;
	_params.rate_i(1) = 0.01;
	_params.rate_d(1) = 0.05;

	/* yaw gains */
	_params.att_p(2) = 3;
	_params.rate_p(2) = 0.1;
	_params.rate_i(2) = 0.00;
	_params.rate_d(2) = 0.0;


	_params.yaw_rate_max = 0.5;
	_params.yaw_ff = 0.5;


	_params.man_roll_max = 0.6;
	_params.man_pitch_max = 0.6;
	_params.man_yaw_max = 0.6;

}

//this function is automatically called if a mavlink attitude message arrives from the PIXHAWK
//this function triggers the main-task function, which calculates the attitude rates setpoint
void MulticopterAttitudeControl::attitude_cb(const simulation::simout msg)
{
	_v_att.roll = msg.roll;
	_v_att.pitch = msg.pitch;
	_v_att.yaw = msg.yaw;
	_v_att.rollspeed = msg.rollspeed;
	_v_att.pitchspeed = msg.pitchspeed;
	_v_att.yawspeed = msg.yawspeed;

//	//calculate Rotation matrix
	math::Matrix<3,3> R;
	R.from_euler(_v_att.roll,_v_att.pitch,_v_att.yaw);

	_v_att.R[0][0] = R(0,0);
	_v_att.R[0][1] = R(0,1);
	_v_att.R[0][2] = R(0,2);
	_v_att.R[1][0] = R(1,0);
	_v_att.R[1][1] = R(1,1);
	_v_att.R[1][2] = R(1,2);
	_v_att.R[2][0] = R(2,0);
	_v_att.R[2][1] = R(2,1);
	_v_att.R[2][2] = R(2,2);
	_v_att.R_valid = true;


task_main(); //run the main task when new attitude message arrives


}

void MulticopterAttitudeControl::rc_cb(const mavros::ManualControl msg)
{

		_manual_control_sp.x = (float)msg.x/1000.f;
		_manual_control_sp.y = (float)msg.y/1000.f;
		_manual_control_sp.z = (float)msg.z/1000.f;
		_manual_control_sp.r = (float)msg.r/1000.f;
	//still have to fill out all other field, like timestamp...
}

void MulticopterAttitudeControl::armed_status_cb(mavros::ArmedState msg)
{

	_armed.armed = msg.armedStatus;

}

void MulticopterAttitudeControl::fill_rates_msg(mavros::AttitudeRates &msg)
{
	msg.rollrate = _v_rates_sp.roll;
	msg.pitchrate = _v_rates_sp.pitch;
	msg.yawrate = _v_rates_sp.yaw;
	msg.thrust = _v_rates_sp.thrust;
}

double MulticopterAttitudeControl::get_dt()
{
	double helper = time_last;
	time_last = ros::Time::now().toSec();
	return ros::Time::now().toSec() - helper;
}

void MulticopterAttitudeControl::mix_and_publish(void)	//this should eventually be in the VTOL folder
{

	simulation::mixer_out msg;
	float result[4] = {0,0,0,0};
	mcMixer->mix(_actuators,result);

	msg.throttle_0 = result[2];
	msg.throttle_1 = result[3];
	msg.throttle_2 = result[0];
	msg.throttle_3 = result[1];

	ROS_INFO("throttle 0 %.5f",msg.throttle_0);

	//mix elevons and guard against to large values (JSBsim takes inputs between -1 and 1)
	msg.aileron = _manual_control_sp.y;
	if (msg.aileron > 1)
	{
		msg.aileron = 1;
	}
	else if (msg.aileron < -1)
	{
		msg.aileron = -1;
	}


	msg.elevator = _manual_control_sp.x;
	if (msg.elevator > 1)
	{
		msg.elevator = 1;
	}
	else if (msg.elevator < -1)
	{
		msg.elevator = -1;
	}



	mixer_out_pub.publish(msg);
}

void MulticopterAttitudeControl::control_attitude(double dt)
{

	float yaw_sp_move_rate = 0.0f;
		bool publish_att_sp = false;

		if (_v_control_mode.flag_control_manual_enabled) {
			/* manual input, set or modify attitude setpoint */

			if (_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_climb_rate_enabled) {
				/* in assisted modes poll 'vehicle_attitude_setpoint' topic and modify it */
				//vehicle_attitude_setpoint_poll();
				//check if even needed in ros with the callback functions
			}

			if (!_v_control_mode.flag_control_climb_rate_enabled) {
				/* pass throttle directly if not in altitude stabilized mode */
				_v_att_sp.thrust = _manual_control_sp.z;
				publish_att_sp = true;
			}

			if (!_armed.armed) {
				/* reset yaw setpoint when disarmed */
				_reset_yaw_sp = true;
			}

			/* move yaw setpoint in all modes */
			if (_v_att_sp.thrust < 0.1f) {
				// TODO
				//if (_status.condition_landed) {
				/* reset yaw setpoint if on ground */
				//	reset_yaw_sp = true;
				//}
			} else {
				/* move yaw setpoint */
				yaw_sp_move_rate = _manual_control_sp.r * _params.man_yaw_max;
				_v_att_sp.yaw_body = _wrap_pi(_v_att_sp.yaw_body + yaw_sp_move_rate * dt);
				float yaw_offs_max = _params.man_yaw_max / _params.att_p(2);
				float yaw_offs = _wrap_pi(_v_att_sp.yaw_body - _v_att.yaw);
				if (yaw_offs < - yaw_offs_max) {
					_v_att_sp.yaw_body = _wrap_pi(_v_att.yaw - yaw_offs_max);

				} else if (yaw_offs > yaw_offs_max) {
					_v_att_sp.yaw_body = _wrap_pi(_v_att.yaw + yaw_offs_max);

				}
				_v_att_sp.R_valid = false;
				publish_att_sp = true;
			}

			/* reset yaw setpint to current position if needed */
			if (_reset_yaw_sp) {
				_reset_yaw_sp = false;
				_v_att_sp.yaw_body = _v_att.yaw;
				_v_att_sp.R_valid = false;
				publish_att_sp = true;
			}

			if (!_v_control_mode.flag_control_velocity_enabled) {
				/* update attitude setpoint if not in position control mode */
				_v_att_sp.roll_body = _manual_control_sp.y * _params.man_roll_max;
				_v_att_sp.pitch_body = -_manual_control_sp.x * _params.man_pitch_max;
				_v_att_sp.R_valid = false;
				publish_att_sp = true;
			}

		} else {
			/* in non-manual mode use 'vehicle_attitude_setpoint' topic */
			//vehicle_attitude_setpoint_poll();
			//check if this is even needed in ros

			/* reset yaw setpoint after non-manual control mode */
			_reset_yaw_sp = true;
		}

		_thrust_sp = _v_att_sp.thrust;

		/* construct attitude setpoint rotation matrix */
		math::Matrix<3, 3> R_sp;

		if (_v_att_sp.R_valid) {
			/* rotation matrix in _att_sp is valid, use it */
			R_sp.set(&_v_att_sp.R_body[0][0]);

		} else {
			/* rotation matrix in _att_sp is not valid, use euler angles instead */
			R_sp.from_euler(_v_att_sp.roll_body, _v_att_sp.pitch_body, _v_att_sp.yaw_body);

			/* copy rotation matrix back to setpoint struct */
			memcpy(&_v_att_sp.R_body[0][0], &R_sp.data[0][0], sizeof(_v_att_sp.R_body));
			_v_att_sp.R_valid = true;
		}

		/* publish the attitude setpoint if needed */
		if (publish_att_sp) {
			mavros::AttitudeRates message;
			fill_rates_msg(message);
			att_rates_pub.publish(message);
		}
		/* rotation matrix for current state */
		math::Matrix<3, 3> R;
		R.set(_v_att.R);
		/* all input data is ready, run controller itself */

		/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
		math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
		math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

		/* axis and sin(angle) of desired rotation */
		math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

		/* calculate angle error */
		float e_R_z_sin = e_R.length();
		float e_R_z_cos = R_z * R_sp_z;

		/* calculate weight for yaw control */
		float yaw_w = R_sp(2, 2) * R_sp(2, 2);

		/* calculate rotation matrix after roll/pitch only rotation */
		math::Matrix<3, 3> R_rp;

		if (e_R_z_sin > 0.0f) {
			/* get axis-angle representation */
			float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
			math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

			e_R = e_R_z_axis * e_R_z_angle;

			/* cross product matrix for e_R_axis */
			math::Matrix<3, 3> e_R_cp;
			e_R_cp.zero();
			e_R_cp(0, 1) = -e_R_z_axis(2);
			e_R_cp(0, 2) = e_R_z_axis(1);
			e_R_cp(1, 0) = e_R_z_axis(2);
			e_R_cp(1, 2) = -e_R_z_axis(0);
			e_R_cp(2, 0) = -e_R_z_axis(1);
			e_R_cp(2, 1) = e_R_z_axis(0);

			/* rotation matrix for roll/pitch only rotation */
			R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

		} else {
			/* zero roll/pitch rotation */
			R_rp = R;
		}

		/* R_rp and R_sp has the same Z axis, calculate yaw error */
		math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
		math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
		e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

		if (e_R_z_cos < 0.0f) {
			/* for large thrust vector rotations use another rotation method:
			 * calculate angle and axis for R -> R_sp rotation directly */
			math::Quaternion q;
			q.from_dcm(R.transposed() * R_sp);
			math::Vector<3> e_R_d = q.imag();
			e_R_d.normalize();
			e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));

			/* use fusion of Z axis based rotation and direct rotation */
			float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
			e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
		}
		/* calculate angular rates setpoint */


		_rates_sp = _params.att_p.emult(e_R);

		/* limit yaw rate */
		_rates_sp(2) = math::constrain(_rates_sp(2), -_params.yaw_rate_max, _params.yaw_rate_max);

		/* feed forward yaw setpoint rate */

		_rates_sp(2) += yaw_sp_move_rate * yaw_w * _params.yaw_ff;


}

void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_armed.armed) {
		_rates_int.zero();

	}

	/* current body angular rates */
	math::Vector<3> rates;
	rates(0) = _v_att.rollspeed;
	rates(1) = _v_att.pitchspeed;
	rates(2) = _v_att.yawspeed;

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	_att_control = _params.rate_p.emult(rates_err) + _params.rate_d.emult(_rates_prev - rates) / dt + _rates_int;
	_rates_prev = rates;

	/* update integral only if not saturated on low limit */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = 0; i < 3; i++) {
			if (fabsf(_att_control(i)) < _thrust_sp) {
				float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

				if ((rate_i) && rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT && //isfinite function should be fixed
				    _att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT) {
					_rates_int(i) = rate_i;
				}
			}
		}
	}
}

void MulticopterAttitudeControl::task_main()
{
	//put condition so that this only runs on attitude changes
	math::Vector<3> rates_err;

	float dt = get_dt();
	if (dt < 0.002f)
	{
		dt = 0.002f;
	}
	else if (dt > 0.02f)
	{
		dt = 0.02f;
	}



	if (_v_control_mode.flag_control_attitude_enabled) {
					control_attitude(dt);

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = 0;	//put time here

					//publish attitude rates
					mavros::AttitudeRates message;
					fill_rates_msg(message);
					att_rates_pub.publish(message);

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					if (_v_control_mode.flag_control_manual_enabled) {
						/* manual rates control - ACRO mode */
						//do something with vector _rates_sp
						//_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x, _manual_control_sp.r).emult(_params.acro_rate_max);
						_thrust_sp = _manual_control_sp.z;

						/* reset yaw setpoint after ACRO */
						_reset_yaw_sp = true;

						/* publish attitude rates setpoint */
						_v_rates_sp.roll = _rates_sp(0);
						_v_rates_sp.pitch = _rates_sp(1);
						_v_rates_sp.yaw = _rates_sp(2);
						_v_rates_sp.thrust = _thrust_sp;
						_v_rates_sp.timestamp = 0;	//put correct timestamp

						//publish attitude rates
						mavros::AttitudeRates message;
						fill_rates_msg(message);
						att_rates_pub.publish(message);

					} else {
						/* attitude controller disabled, poll rates setpoint topic */
						//vehicle_rates_setpoint_poll();   //need to see if can call callback manually in ros
						_rates_sp(0) = _v_rates_sp.roll;
						_rates_sp(1) = _v_rates_sp.pitch;
						_rates_sp(2) = _v_rates_sp.yaw;
						_thrust_sp = _v_rates_sp.thrust;
					}
				}
				if (_v_control_mode.flag_control_rates_enabled) {
						control_attitude_rates(dt);

						/* publish actuator controls */
	//					_actuators.control[0] = (isfinite(_att_control(0))) ? _att_control(0) : 0.0f;
	//					_actuators.control[1] = (isfinite(_att_control(1))) ? _att_control(1) : 0.0f;
	//					_actuators.control[2] = (isfinite(_att_control(2))) ? _att_control(2) : 0.0f;
	//					_actuators.control[3] = (isfinite(_thrust_sp)) ? _thrust_sp : 0.0f;
	//					_actuators.timestamp = hrt_absolute_time();
						//need to implement isfinite again, this is just for testing!!!!!!!!!!!!!!!
						_actuators.control[0] = _att_control(0);
						_actuators.control[1] = _att_control(1);
						_actuators.control[2] = _att_control(2);
						_actuators.control[3] = _thrust_sp;
						_actuators.timestamp = 0; //put correct time here

						if (!_actuators_0_circuit_breaker_enabled) {
							mix_and_publish();

						}
					}

			}

int main(int argc, char **argv)
{

	ros::init(argc,argv,"mc_att_controller_sim");

	MulticopterAttitudeControl AttController;
	ros::spin();

	return 0;


}
