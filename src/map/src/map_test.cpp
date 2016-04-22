#include "ros/ros.h"
#include <vector>
#include <cstdint>
#include <iostream>
#include <cstddef>

#include "map/map_config.h"
#include "map/cross_msg.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_config_test");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<map::map_config>("get_map_config");
	map::map_config srv;

	uint8_t ReqID = 0;
	srv.request.req = (uint8_t) ReqID;
	
	if (client.call(srv)) {
		ROS_INFO("Calling for map config with request: %d. Number of crossings: %d, vector size: %d", (int16_t) ReqID, (int16_t)srv.response.number_of_crossings, (int16_t) srv.response.crossings.size());
	} else {
		ROS_ERROR("Failed to call map for config");
		return 1;
	}

	return 0;
}
