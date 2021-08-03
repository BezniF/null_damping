#include "null_damping.h"
#include <signal.h>

void mySigintHandler(int sig){
	ros::NodeHandle nh;

	ros::Publisher arm_pose_pub;
	arm_pose_pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

	std_msgs::Float64MultiArray stop_msg;
	std::vector<double> stop_vector;
	stop_vector.resize(6, 0.0);

	stop_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	stop_msg.layout.dim[0].size = stop_vector.size();
	stop_msg.layout.dim[0].stride = 1;
	stop_msg.layout.dim[0].label = "stop_vel";

	stop_msg.data.clear();
	stop_msg.data.insert(stop_msg.data.end(), stop_vector.begin(), stop_vector.end());

	arm_pose_pub.publish(stop_msg);

	ros::shutdown();
}

int main(int argc, char **argv){

		ros::init(argc, argv, "null_damping",ros::init_options::NoSigintHandler);
		NullDamping* nd = new NullDamping();

		signal(SIGINT, mySigintHandler);

		ros::Rate r(500); // Was 500
		while(ros::ok()) {

			ros::spinOnce();
			nd->spin();
			r.sleep();
		}
		
		delete nd;

//return 0;

}