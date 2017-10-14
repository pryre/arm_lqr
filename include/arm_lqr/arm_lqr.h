#pragma once

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

class ArmLQR {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;
		ros::Publisher pub_control_;
		ros::Subscriber sub_state_;

		std::string param_topic_states_;
		std::string param_topic_setpoints_;
		double param_control_rate_;

		bool flag_got_state_;

		sensor_msgs::JointState state_;
		std::vector<std::vector<double>> K_;
		std::vector<double> yf_;

	public:
		ArmLQR( void );

		~ArmLQR( void );

		void callback_control(const ros::TimerEvent& e);
		void callback_state(const sensor_msgs::JointState::ConstPtr& msg_in);
};
