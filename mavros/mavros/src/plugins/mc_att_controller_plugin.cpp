//@author: Roman Bapst

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros/Attitude.h>
#include <mavros/ManualControl.h>
#include <mavros/ArmedState.h>

namespace mavplugin {

/**
 * @brief plugin for the PX4 mc_attitude_controller module running offboard
 */
class mc_att_controllerPlugin : public MavRosPlugin {
public:
	mc_att_controllerPlugin() :
		uas(nullptr)
{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;


		imu_attitude_pub 	= nh.advertise<mavros::Attitude>("imu/Attitude",10);
		manual_setpoint_pub = nh.advertise<mavros::ManualControl>("ManualSetpoint",10);
		armed_state_pub 	= nh.advertise<mavros::ArmedState>("ArmedState",10);
//
	}

	std::string const get_name() const {
		return "mc_att_controllerPlugin";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE, &mc_att_controllerPlugin::handle_attitude),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_MANUAL_CONTROL, &mc_att_controllerPlugin::handle_manual_setpoint),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_HEARTBEAT, &mc_att_controllerPlugin::handle_armed_state),
		};
	}

private:
	std::string frame_id;
	UAS *uas;

	ros::Publisher imu_attitude_pub;
	ros::Publisher manual_setpoint_pub;
	ros::Publisher armed_state_pub;


	/* -*- message handlers -*- */

	void handle_attitude(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

		mavlink_attitude_t att;
		mavlink_msg_attitude_decode(msg, &att);
		mavros::Attitude msg_att;
		msg_att.roll 		= att.roll;
		msg_att.pitch 		= att.pitch;
		msg_att.yaw 		= att.yaw;
		msg_att.rollspeed 	= att.rollspeed;
		msg_att.pitchspeed 	= att.pitchspeed;
		msg_att.yawspeed 	= att.yawspeed;
		imu_attitude_pub.publish(msg_att);
	}

	void handle_manual_setpoint(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_manual_control_t Controls;
		mavlink_msg_manual_control_decode(msg,&Controls);
		mavros::ManualControl message;
		message.x = Controls.x;
		message.y = Controls.y;
		message.z = Controls.z;
		message.r = Controls.r;
		manual_setpoint_pub.publish(message);
	}

	void handle_armed_state(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);
		mavros::ArmedState message;
		message.armedStatus = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
		armed_state_pub.publish(message);
		}

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::mc_att_controllerPlugin, mavplugin::MavRosPlugin)
