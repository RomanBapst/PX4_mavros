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
#include <cmath>
#include <math.h>
#include <time.h>

#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <modules/mc_att_control/mc_att_control_base.h>
#include <drivers/drv_hrt.h>

#include <ros/ros.h>

#include <mavros/ManualControl.h>
#include <mavros/AttitudeRates.h>
#include <mavros/ArmedState.h>
#include <mavros/Attitude.h>
#include <mavros/AttitudeSetpoint.h>
#include <mavros/Actuator.h>

using namespace std;


class MulticopterAttitudeControl: public MulticopterAttitudeControlBase {
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
	ros::Publisher _att_pub;

	//Class methods
	void attitude_cb(const mavros::Attitude msg); //called when new attitude message arrives
	void rc_cb(const mavros::ManualControl msg); //called when new manual attitude setpoint arrives
	void armed_status_cb(mavros::ArmedState msg); //called when new heartbeat message arrives (contains armed status)
	void fill_rates_msg(mavros::AttitudeRates &msg); //fill message with attitude rates
	void task_main(); 						//this is called by 'attitude_cb'

};

MulticopterAttitudeControl::MulticopterAttitudeControl() {
	//setup all subscribers
	attitude_sub = n.subscribe("mavros/imu/Attitude", 1000,
			&MulticopterAttitudeControl::attitude_cb, this);
	rc_sub = n.subscribe("mavros/ManualSetpoint", 1000,
			&MulticopterAttitudeControl::rc_cb, this);
	armed_status_sub = n.subscribe("mavros/ArmedState", 1000,
			&MulticopterAttitudeControl::armed_status_cb, this);

	//setup all publishers
	att_rates_pub = n.advertise<mavros::AttitudeRates>(
			"MulticopterAttitudeControl/att_rates", 10);
	_att_pub = n.advertise<mavros::AttitudeSetpoint>(
			"MulticopterAttitudeControl/att_setpoint", 10);
	_actuators_0_pub = n.advertise<mavros::Actuator>(
			"MulticopterAttitudeControl/actuators_0", 10);

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
	_params.att_p(0) = 7.0;
	_params.rate_p(0) = 0.1;
	_params.rate_i(0) = 0.0;
	_params.rate_d(0) = 0.003;

	/* pitch gains */
	_params.att_p(1) = 7.0;
	_params.rate_p(1) = 0.1;
	_params.rate_i(1) = 0.0;
	_params.rate_d(1) = 0.003;

	/* yaw gains */
	_params.att_p(2) = 2.8;
	_params.rate_p(2) = 0.2;
	_params.rate_i(2) = 0.1;
	_params.rate_d(2) = 0.0;

	_params.yaw_rate_max = 0.5;
	_params.yaw_ff = 0.5;

	_params.man_roll_max = 0.6;
	_params.man_pitch_max = 0.6;
	_params.man_yaw_max = 0.6;

}

//this function is automatically called if a mavlink attitude message arrives from the PIXHAWK
//this function triggers the main-task function, which calculates the attitude rates setpoint
void MulticopterAttitudeControl::attitude_cb(const mavros::Attitude msg) {
	_v_att.roll = msg.roll;
	_v_att.pitch = msg.pitch;
	_v_att.yaw = msg.yaw;
	_v_att.rollspeed = msg.rollspeed;
	_v_att.pitchspeed = msg.pitchspeed;
	_v_att.yawspeed = msg.yawspeed;

//	//calculate Rotation matrix
	math::Matrix<3, 3> R;
	R.from_euler(_v_att.roll, _v_att.pitch, _v_att.yaw);

	_v_att.R[0][0] = R(0, 0);
	_v_att.R[0][1] = R(0, 1);
	_v_att.R[0][2] = R(0, 2);
	_v_att.R[1][0] = R(1, 0);
	_v_att.R[1][1] = R(1, 1);
	_v_att.R[1][2] = R(1, 2);
	_v_att.R[2][0] = R(2, 0);
	_v_att.R[2][1] = R(2, 1);
	_v_att.R[2][2] = R(2, 2);
	_v_att.R_valid = true;

	task_main(); //run the main task

}

void MulticopterAttitudeControl::rc_cb(const mavros::ManualControl msg) {
	_manual_control_sp.x = (float) msg.x / 1000.f;
	_manual_control_sp.y = (float) msg.y / 1000.f;
	_manual_control_sp.z = (float) msg.z / 1000.f;
	_manual_control_sp.r = (float) msg.r / 1000.f;
	//still have to fill out all other field, like timestamp...
}

void MulticopterAttitudeControl::armed_status_cb(mavros::ArmedState msg) {
	_armed.armed = msg.armedStatus;
}

void MulticopterAttitudeControl::fill_rates_msg(mavros::AttitudeRates &msg) {
	msg.rollrate = _v_rates_sp.roll;
	msg.pitchrate = _v_rates_sp.pitch;
	msg.yawrate = _v_rates_sp.yaw;
	msg.thrust = _v_rates_sp.thrust;
}

void MulticopterAttitudeControl::task_main() {
	static uint64_t last_run = 0;
	float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	/* guard against too small (< 2ms) and too large (> 20ms) dt's */
	if (dt < 0.002f) {
		dt = 0.002f;

	} else if (dt > 0.02f) {
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
		_actuators.control[0] =
				(isfinite(_att_control(0))) ? _att_control(0) : 0.0f;
		_actuators.control[1] =
				(isfinite(_att_control(1))) ? _att_control(1) : 0.0f;
		_actuators.control[2] =
				(isfinite(_att_control(2))) ? _att_control(2) : 0.0f;
		_actuators.control[3] = (isfinite(_thrust_sp)) ? _thrust_sp : 0.0f;
		//_actuators.timestamp = hrt_absolute_time();

		if (!_actuators_0_circuit_breaker_enabled) {
			//publish the actuator commands in here!!!
		}
	}
	perf_end (_loop_perf);
}

int main(int argc, char **argv) {
//intialize node
	ros::init(argc, argv, "mc_att_controller");

//create instance of the node
	MulticopterAttitudeControl AttController;

	ros::spin();

	return 0;

}
