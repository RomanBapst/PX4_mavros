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
 * @file fw_att_control_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>

#include <ros/ros.h>
//#include <ros_error.h>	//this should also be replaced by systemlib/err.h
#include <modules/fw_att_control/fw_att_control_base.h>

#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

#include <mavros/ManualControl.h>
#include <mavros/AttitudeRates.h>
#include <mavros/AttitudeSetpoint.h>
#include <mavros/AttitudeRateSetpoint.h>
#include <mavros/Attitude.h>
#include <mavros/Airspeed.h>
#include <mavros/Actuator.h>

using namespace std;	//this is needed for the isfinite function call
/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */

class FixedwingAttitudeControl: public FixedwingAttitudeControlBase
{
public:
	/**
	 * Constructor
	 */
	FixedwingAttitudeControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingAttitudeControl(){};

private:
	ros::NodeHandle n;
	ros::Subscriber attitude_sub;
	ros::Subscriber manual_control_sub;
	ros::Subscriber airspeed_sub;

	ros::Publisher _actuators_virtual_5_pub;
	ros::Publisher _attitude_sp_pub;
	ros::Publisher _rate_sp_pub;

	double time_last;	//used to measure elapsed time

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */


/////////////// Member Functions ///////////////////////////////////////////////////////////////////
	void		task_main();
	void 		attitude_cb(mavros::Attitude msg);
	void 		manual_control_cb(mavros::ManualControl msg);
	void 		airspeed_cb(mavros::Airspeed msg);
	void		fill_attitude_sp_msg(mavros::AttitudeSetpoint &msg,struct vehicle_attitude_setpoint_s &att_sp);
	void		fill_attitude_rate_sp_msg(mavros::AttitudeRateSetpoint &msg, struct vehicle_rates_setpoint_s &rates_sp);
	void 		fill_actuator_msg(mavros::Actuator &msg,struct actuator_controls_s &actuators);


};



FixedwingAttitudeControl::FixedwingAttitudeControl()
	{

	//setup all subscribers
	attitude_sub 		= n.subscribe("mavros/imu/Attitude",1000,&FixedwingAttitudeControl::attitude_cb,this);
	manual_control_sub 	= n.subscribe("mavros/ManualSetpoint",1000,&FixedwingAttitudeControl::manual_control_cb,this);
	airspeed_sub 		= n.subscribe("mavros/Airspeed",1000,&FixedwingAttitudeControl::airspeed_cb,this);

	//setup all publishers
	_actuators_virtual_5_pub 	= n.advertise<mavros::Actuator>("FixedwingAttitudeControl/actuators_virtual_5",10);
	_attitude_sp_pub 			= n.advertise<mavros::AttitudeSetpoint>("FixedwingAttitudeControl/attitude_setpoint",10);
	_rate_sp_pub 				= n.advertise<mavros::AttitudeRateSetpoint>("FixedwingAttitudeControl/attitude_rate_setpoint",10);

	//setup time variable, this is used by the class-method 'get_dt()'
	time_last = ros::Time::now().toSec();



	_vcontrol_mode.flag_armed 						= true;
	_vcontrol_mode.flag_control_altitude_enabled 	= false;
	_vcontrol_mode.flag_control_attitude_enabled 	= true;
	_vcontrol_mode.flag_control_auto_enabled		= false;
	_vcontrol_mode.flag_control_climb_rate_enabled 	= false;
	_vcontrol_mode.flag_control_manual_enabled 		= true;
	_vcontrol_mode.flag_control_offboard_enabled 	= false;
	_vcontrol_mode.flag_control_position_enabled 	= false;
	_vcontrol_mode.flag_control_rates_enabled 		= true;
	_vcontrol_mode.flag_control_termination_enabled = false;
	_vcontrol_mode.flag_control_velocity_enabled	= false;
	_vcontrol_mode.flag_external_manual_override_ok	= false;
	_vcontrol_mode.flag_system_hil_enabled			= false;

	//initialize parameters struct ...
	memset(&_parameters,0,sizeof(_parameters));
	_parameters.tconst = 0.1;	//400 Hz
	_parameters.p_p = 0.3;
	_parameters.p_d = 0;
	_parameters.p_i = 0.01;
	_parameters.p_ff = 0.2;
	_parameters.p_rmax_pos = 0;
	_parameters.p_rmax_neg = 0;
	_parameters.p_integrator_max = 1;
	_parameters.p_roll_feedforward = 0.3;
	_parameters.r_p = 1;
	_parameters.r_d = 0;
	_parameters.r_i = 0.01;
	_parameters.r_ff = 0.3;
	_parameters.r_integrator_max = 1;
	_parameters.r_rmax = 0.0;
	_parameters.y_p = 0.5;
	_parameters.y_i = 0;
	_parameters.y_d = 0;
	_parameters.y_ff = 0.3;
	_parameters.y_roll_feedforward = 0.3;
	_parameters.y_integrator_max = 1;
	_parameters.y_coordinated_min_speed = 0;
	_parameters.y_rmax = 0.3;

	_parameters.airspeed_min = 5;
	_parameters.airspeed_trim = 10;
	_parameters.airspeed_max = 20;

	_parameters.trim_roll = 0;
	_parameters.trim_pitch = 0;
	_parameters.trim_yaw = 0;
	_parameters.rollsp_offset_deg = 0;			/**< Roll Setpoint Offset in deg */
	_parameters.pitchsp_offset_deg = 0;			/**< Pitch Setpoint Offset in deg */
	_parameters.rollsp_offset_rad = 0;			/**< Roll Setpoint Offset in rad */
	_parameters.pitchsp_offset_rad = 0;
	_parameters.man_pitch_max = 0.6;
	_parameters.man_roll_max = 0.6;

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.tconst);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));
	_pitch_ctrl.set_roll_ff(_parameters.p_roll_feedforward);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.tconst);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));
}


//this function is automatically called if a mavlink attitude message arrives from the PIXHAWK
//this function triggers the main-task function, which calculates the attitude rates setpoint
void FixedwingAttitudeControl::attitude_cb(const mavros::Attitude msg)
{
	_att.timestamp = ros::Time::now().toNSec() / 1e3;
	_att.roll 		= msg.roll;
	_att.pitch 		= msg.pitch;
	_att.yaw 		= msg.yaw;
	_att.rollspeed 	= msg.rollspeed;
	_att.pitchspeed = msg.pitchspeed;
	_att.yawspeed 	= msg.yawspeed;

//	//calculate Rotation matrix
	math::Matrix<3,3> R;
	R.from_euler(_att.roll,_att.pitch,_att.yaw);

	_att.R[0][0] = R(0,0);
	_att.R[0][1] = R(0,1);
	_att.R[0][2] = R(0,2);
	_att.R[1][0] = R(1,0);
	_att.R[1][1] = R(1,1);
	_att.R[1][2] = R(1,2);
	_att.R[2][0] = R(2,0);
	_att.R[2][1] = R(2,1);
	_att.R[2][2] = R(2,2);
	_att.R_valid = true;


task_main(); //run the main task


}

void FixedwingAttitudeControl::manual_control_cb(const mavros::ManualControl msg)
{
	_manual.timestamp = ros::Time::now().toNSec() / 1e3;
	_manual.x = (float)msg.x/1000.f;
	_manual.y = (float)msg.y/1000.f;
	_manual.z = (float)msg.z/1000.f;
	_manual.r = (float)msg.r/1000.f;
	//still have to fill out all other field, like timestamp...
}



void FixedwingAttitudeControl::airspeed_cb(mavros::Airspeed msg)
{
	_airspeed.timestamp = ros::Time::now().toNSec() / 1e3;
	_airspeed.true_airspeed_m_s 		= msg.true_airspeed_m_s;
	_airspeed.indicated_airspeed_m_s 	= msg.indicated_airspeed_m_s;
	_airspeed.air_temperature_celsius 	= msg.air_temperature_celsius;
}

void FixedwingAttitudeControl::fill_attitude_sp_msg(mavros::AttitudeSetpoint &msg,struct vehicle_attitude_setpoint_s &att_sp)
{
	msg.roll = att_sp.roll_body;
	msg.pitch = att_sp.pitch_body;
	msg.yaw  = att_sp.yaw_body;
	msg.thrust = att_sp.thrust;

}

void FixedwingAttitudeControl::fill_attitude_rate_sp_msg(mavros::AttitudeRateSetpoint &msg, struct vehicle_rates_setpoint_s &rates_sp)
{
	msg.rollrate = rates_sp.roll;
	msg.pitchrate = rates_sp.pitch;
	msg.yawrate  = rates_sp.yaw;

}

void FixedwingAttitudeControl::fill_actuator_msg(mavros::Actuator &msg,struct actuator_controls_s &actuators)
{
	msg.roll = actuators.control[0];
	msg.pitch = actuators.control[1];
	msg.yaw = actuators.control[2];
	msg.thrust = actuators.control[3];

}

void
FixedwingAttitudeControl::task_main()
{

	static int loop_counter = 0;
	perf_begin(_loop_perf);


	static uint64_t last_run = 0;
	float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	/* guard against too large deltaT's */
	if (deltaT > 1.0f)
		deltaT = 0.01f;


	/* Simple handling of failsafe: deploy parachute if failsafe is on */
	if (_vcontrol_mode.flag_control_termination_enabled) {
		_actuators_airframe.control[1] = 1.0f;

	} else {
		_actuators_airframe.control[1] = 0.0f;
	}

	/* decide if in stabilized or full manual control */

	if (_vcontrol_mode.flag_control_attitude_enabled) {
			control_attitude();

		/*
		 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
		 * only once available
		 */
		vehicle_rates_setpoint_s rates_sp;
		rates_sp.roll = _roll_ctrl.get_desired_rate();
		rates_sp.pitch = _pitch_ctrl.get_desired_rate();
		rates_sp.yaw = _yaw_ctrl.get_desired_rate();

		//publish attitude setpoint
		mavros::AttitudeRateSetpoint message;
		fill_attitude_rate_sp_msg(message,rates_sp);
		_rate_sp_pub.publish(message);

	} else {
		/* manual/direct control */
		_actuators.control[0] = _manual.y;
		_actuators.control[1] = -_manual.x;
		_actuators.control[2] = _manual.r;
		_actuators.control[3] = _manual.z;
		_actuators.control[4] = _manual.flaps;
	}

	_actuators.control[5] = _manual.aux1;
	_actuators.control[6] = _manual.aux2;
	_actuators.control[7] = _manual.aux3;

	//publish the actuator setpoints
	mavros::Actuator message;
	fill_actuator_msg(message,_actuators);
	_actuators_virtual_5_pub.publish(message);
}



int main(int argc, char *argv[])
{
	//intialize node
	ros::init(argc,argv,"fw_att_controller");

	//create instance of the node
	FixedwingAttitudeControl AttController;

	ros::spin();

	return 0;
}
