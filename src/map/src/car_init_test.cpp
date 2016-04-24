#include "ros/ros.h"
#include <vector>
#include <cstdint>
#include <iostream>
#include <cstddef>

#include "map/car_init.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_car_init_test");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<map::car_init>("init_car");
	map::car_init srv;

	uint8_t ReqID = 0;
	srv.request.req = (uint8_t) ReqID;
	
	if (client.call(srv)) {
		ROS_INFO("Calling for map init with request: %d. Got ID %d, path %d -> %d of length %d", (int16_t) ReqID, (int16_t)srv.response.carID, (int16_t) srv.response.prevCrossing, (int16_t) srv.response.nextCrossing, (int16_t) srv.response.pathLenght);
	} else {
		ROS_ERROR("Failed to call map for config");
		return 1;
	}

	return 0;
}
