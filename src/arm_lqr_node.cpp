#include <ros/ros.h>
#include <arm_lqr/arm_lqr.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "spinner_cpp");
	ArmLQR ac;

	ros::spin();

	return 0;
}
