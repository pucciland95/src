#include "../include/controller.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_controller");
	ros::NodeHandle nh;

	// Create the controller specific to your robot
	std::shared_ptr<Controller> p_controller = std::make_shared<Controller>(nh);

	ros::spin();
	
	return 0;
}
