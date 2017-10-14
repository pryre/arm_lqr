#include <ros/ros.h>

#include <arm_lqr/arm_lqr.h>

#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <math.h>

ArmLQR::ArmLQR() :
	nh_("~"),
	param_topic_states_("state"),
	param_topic_setpoints_("setpoint"),
	param_control_rate_(25.0),
	flag_got_state_(false) {

	nh_.param("topic_input_states", param_topic_states_, param_topic_states_);
	nh_.param("topic_output_setpoints", param_topic_setpoints_, param_topic_setpoints_);
	nh_.param("control_rate", param_control_rate_, param_control_rate_);

	//Load in K matrix
	ROS_INFO("Loading K LQR gains");

	//TODO: work for any size vector
	for(int j=0; j<2; j++) {
		std::vector<double> vec;

		nh_.param<std::vector<double>>("k_gains/k" + std::to_string(j+1), vec, vec);

		K_.push_back(vec);

		ROS_INFO("[%0.2f, %0.2f, %0.2f, %0.2f]", K_[j][0], K_[j][1], K_[j][2], K_[j][3]);
	}

	//Load goal state
	ROS_INFO("Loading yf");
	nh_.param<std::vector<double>>("yf", yf_, yf_);
	ROS_INFO("[%0.2f, %0.2f, %0.2f, %0.2f]", yf_[0], yf_[1], yf_[2], yf_[3]);

	pub_control_ = nh_.advertise<sensor_msgs::JointState>(param_topic_setpoints_, 10);
	sub_state_ = nh_.subscribe<sensor_msgs::JointState>( param_topic_states_, 10, &ArmLQR::callback_state, this );

	timer_ = nh_.createTimer(ros::Duration(1.0/param_control_rate_), &ArmLQR::callback_control, this );

	ROS_INFO("Started LQR Controller!");

}

ArmLQR::~ArmLQR() {
}

void ArmLQR::callback_state(const sensor_msgs::JointState::ConstPtr& msg_in) {
	state_ = *msg_in;
	flag_got_state_ = true;
}

void ArmLQR::callback_control(const ros::TimerEvent& e) {
	if(flag_got_state_) {
		std::vector<double> q = state_.position;
		std::vector<double> qd = state_.velocity;
		std::vector<double> u;
		std::vector<double> v;	//v = -K*(yf - y)	//acceleration goals

		//XXX: MOTOR ROTATION OFFSET (REMOVE)
		q[0] += M_PI / 2;


		std::vector<double> y = {q[0], qd[0], q[1], qd[1]};
		std::vector<double> dy;

		for(int i=0; i<y.size(); i++) {
			dy.push_back(y[i] - yf_[i]);
		}

		for(int j=0; j<K_.size(); j++) {
			double sum = 0.0;

			for(int k=0; k<dy.size(); k++) {
				sum += -K_[j][k]*dy[k];
			}

			v.push_back(sum);
		}

		double l1 = 0.239;	//0.068 + 0.102 + 0.069
		double l2 = 0.183;	//0.068 + 0.115
		double lc1 = l1/2.0;
		double lc2 = l2/2.0;
		double m1 = 0.450;	//50g + 50g + 135g
		double m2 = 0.100;	//100g + 100g
		//double Iz1 = 0.0022;
		//double Iz2 = 0.00073;
		double g = -9.80665;
		//double damp = 0.05;
		//double fric = 0.1;
		//double g = 0;
		double Iz1 = 0.042;
		double Iz2 = 0.0043;
		double damp = 0.05;
		double fric1 = 0.2;
		double fric2 = 0.08;

		double v1 = v[0];
		double v2 = v[1];
		double q1 = q[0];
		double q2 = q[1];
		double qd1 = qd[0];
		double qd2 = qd[1];

		// gear ratio:  353.5 : 1
		//double g_ratio = 1/353.5;
		double Ig = 0.0; //Gear inertial
		//double Ie = g_ratio*g_ratio*Ig; //Effective Inertia;

		ROS_INFO("[v: %0.2f, %0.2f]", v1, v2);

		g = 1.0*g;


		double u1 = v1*(Iz1 + Iz2 + l1*l1*m2 + lc1*lc1*m1 + lc2*lc2*m2 + 2*l1*lc2*m2*cos(q2)) + v2*(Iz2 + lc2*lc2*m2 + l1*lc2*m2*cos(q2)) + qd1*damp + g*(lc2*m2*cos(q1 + q2) + l1*m2*cos(q1) + lc1*m1*cos(q1)) - l1*lc2*m2*qd2*qd2*sin(q2) - 2*l1*lc2*m2*qd1*qd2*sin(q2);
		double u2 = v1*(Iz2 + lc2*lc2*m2 + l1*lc2*m2*cos(q2)) + v2*(Iz2 + lc2*lc2*m2 + Ig) + g*(lc2*m2*cos(q1 + q2)) + l1*lc2*m2*qd1*qd1*sin(q2);

		/*
		//Account for motor friction
		//https://www.ais.uni-bonn.de/papers/RC13_Schwarz.pdf (p.4)
		//https://github.com/RhobanProject/Dynaban/blob/master/firmware/trajectory_manager.cpp

		double kvis = -0.0811;  // in N.m.s/rad
		double linearTrans = 0.195;  // linearTransition in rad/s.
		double kstat = 0.12;  // N.m. Minimum torque needed to start moving the motor
		double kcoul = 0.103; //in N.m value of the friction torque when speed = linearTransition.


		double vel = qd2;	//XXX
		double signVel = vel < 0 ? -1 : 1;
		double beta = exp(-fabs( vel / linearTrans)); // range [0, 1]
		//tau_c -> coulombContribution
		double tau_c = (kvis * linearTrans - exp(-1) * kstat + kcoul) / (1 - exp(-1)); // N.m
		//tau_f -> friction torque
		// kvis is negative in this convention
		double tau_f = vel*kvis - signVel * (beta * kstat + (1 - beta) * tau_c);


		ROS_INFO("[tau_f: %0.2f]", tau_f);
		u2 += signDir*fabs(tau_f);
		*/


		/*
		double fric = 0.15;
		double sgnu = v1 > 0 ? 1 : v1 < 0 ? -1: 0;
		double dx = fabs(qd1) < 0.2 ? 1 : 0;
		double fm = sgnu*dx*fric;

		u1 += fm;
		*/



		//XXX: Apply some extra torqe to overcome the

		//double bump1 = 0.2/(exp(fabs(dy[1]*dy[1])));
		//double bump2 = 0.08/(exp(fabs(dy[3]*dy[3])));
		double kstat1 = 0.10;
		double kscale = 1.0;
		double kstat2 = 0.10;

		double signDir1 = -dy[0] < 0 ? -1 : 1;
		double signDir2 = -dy[2] < 0 ? -1 : 1;

		//double bump1 = -dy[0]/(exp(qd1));
		double bump1 = signDir1*kstat1/(exp(fabs(qd1)))*(exp(fabs(dy[0]))-1);
		double bump2 = signDir2*kstat2/(exp(fabs(qd2)))*(exp(fabs(dy[2]))-1);

		bump1 = bump1 > kstat1 ? kstat1 : bump1 < -kstat1 ? -kstat1 : bump1;
		bump2 = bump2 > kstat2 ? kstat2 : bump2 < -kstat2 ? -kstat2 : bump2;

		ROS_INFO("[bump: %0.2f; %0.2f]", bump1, bump2);

		u1 += bump1;
		u2 += bump2;

		//u1 += dy[0] > 0.05 ? -fric1 : dy[0] < -0.05 ? fric1 : 0;
		//u1 += dy[0] > 0.2 ? -bump1 : dy[0] < -0.2 ? bump1 : 0;
		//u2 += dy[2] > 0.2 ? -bump2 : dy[2] < -0.2 ? bump2 : 0;

		//ROS_INFO("[u: %0.2f, %0.2f]", u1, u2);

		u.push_back(u1);
		u.push_back(u2);

		sensor_msgs::JointState msg_out;
		msg_out.header.stamp = ros::Time::now();
		msg_out.header.frame_id = state_.header.frame_id;

		msg_out.name = state_.name;
		msg_out.effort = u;

		pub_control_.publish(msg_out);
	}
}










