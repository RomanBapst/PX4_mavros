/*
 * mc_mixer.cpp
 *
 *  Created on: Sep 17, 2014
 *      Author: roman
 */

#include <mc_mixer/mixer_out.h>
#include <mavros/Actuator.h>

#include <mathlib/mathlib.h>
#include <ros/ros.h>


class MultirotorMixer {
public:

	MultirotorMixer();

	struct Rotor {
			float roll_scale;
			float pitch_scale;
			float yaw_scale;

		};

private:
	ros::NodeHandle n;
	ros::Subscriber _mixer_in_sub;
	ros::Publisher _mixer_out_pub;


	const Rotor *_rotors;

	unsigned _rotor_count;

	struct {
		float control[4];
	}inputs;

	struct  {
		float control[4];
	}outputs;






	void mix();
	void data_in_callback(const mavros::Actuator &msg);
	void task_main();

};

/*
 const MultirotorMixer::Rotor _config_quadshot[] =
 {
 { -0.3223,  0.9466,   0.4242 },
 {  0.3223, -0.9466,   1.0000 },
 {  0.3223,  0.9466,  -0.4242 },
 { -0.3223, -0.9466,  -1.0000 },
 };
 */

const MultirotorMixer::Rotor _config_quadshot[] = {
			{ -1.000000, 0.000000, 1.00 }, { 1.000000, 0.000000, 1.00 }, { 0.000000,
					1.000000, -1.00 }, { -0.000000, -1.000000, -1.00 }, };

	const MultirotorMixer::Rotor *_config_index = { &_config_quadshot[0]

	};

MultirotorMixer::MultirotorMixer() :
		_rotor_count(4), _rotors(_config_index) {

	_mixer_in_sub = n.subscribe("MulticopterAttitudeControl/actuators_0",1000,&MultirotorMixer::data_in_callback,this);
	_mixer_out_pub = n.advertise<mc_mixer::mixer_out>("MulticopterMixer/mixer_out",10);

	memset(&inputs,0,sizeof(inputs));
	memset(&outputs,0,sizeof(outputs));

}

void MultirotorMixer::data_in_callback(const mavros::Actuator &msg) {
	inputs.control[0] = msg.roll;
	inputs.control[1] = msg.pitch;
	inputs.control[2] = msg.yaw;
	inputs.control[3] = msg.thrust;

	task_main();

}

void MultirotorMixer::mix() {
	float roll = math::constrain(inputs.control[0], -1.0f, 1.0f);
	float pitch = math::constrain(inputs.control[1], -1.0f, 1.0f);
	float yaw = math::constrain(inputs.control[2], -1.0f, 1.0f);
	float thrust = math::constrain(inputs.control[3], 0.0f, 1.0f);
	float min_out = 0.0f;
	float max_out = 0.0f;

	/* perform initial mix pass yielding unbounded outputs, ignore yaw */
	for (unsigned i = 0; i < _rotor_count; i++) {
		float out = roll * _rotors[i].roll_scale
				+ pitch * _rotors[i].pitch_scale + thrust;

		/* limit yaw if it causes outputs clipping */
		if (out >= 0.0f && out < -yaw * _rotors[i].yaw_scale) {
			yaw = -out / _rotors[i].yaw_scale;
		}

		/* calculate min and max output values */
		if (out < min_out) {
			min_out = out;
		}
		if (out > max_out) {
			max_out = out;
		}

		outputs.control[i] = out;
	}
	/* scale down roll/pitch controls if some outputs are negative, don't add yaw, keep total thrust */
	if (min_out < 0.0f) {
		float scale_in = thrust / (thrust - min_out);

		/* mix again with adjusted controls */
		for (unsigned i = 0; i < _rotor_count; i++) {
			outputs.control[i] = scale_in
					* (roll * _rotors[i].roll_scale
							+ pitch * _rotors[i].pitch_scale) + thrust;
		}

	} else {
		/* roll/pitch mixed without limiting, add yaw control */
		for (unsigned i = 0; i < _rotor_count; i++) {
			outputs.control[i] += yaw * _rotors[i].yaw_scale;
		}
	}

	/* scale down all outputs if some outputs are too large, reduce total thrust */
	float scale_out;
	if (max_out > 1.0f) {
		scale_out = 1.0f / max_out;

	} else {
		scale_out = 1.0f;
	}

	/* scale outputs to range _idle_speed..1, and do final limiting */
	for (unsigned i = 0; i < _rotor_count; i++) {
		outputs.control[i] = math::constrain(outputs.control[i], 0.0f, 1.0f);
	}

}

void MultirotorMixer::task_main() {
	mix();
	mc_mixer::mixer_out message;
	message.throttle_0 = outputs.control[0];
	message.throttle_1 = outputs.control[1];
	message.throttle_2 = outputs.control[2];
	message.throttle_3 = outputs.control[3];

	_mixer_out_pub.publish(message);


}

int main(int argc, char **argv) {

	ros::init(argc, argv, "MulticopterMixer");

	MultirotorMixer Mixer;

	ros::spin();

	return 0;

}

