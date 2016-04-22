#include "ros/ros.h"
#include <vector>
#include <cstdint>
#include <iostream>
#include <cstddef>

#include "map/map_config.h"
#include "map/cross_msg.h"

bool getConfig(map::map_config::Request  &req,
         map::map_config::Response &res);

class Crossing {

};

class Map
{
	Map() {}
	Map(const Map&) {}
	
	std::vector<Crossing*> crossings = *new std::vector<Crossing*>();


public:
	static Map& getInstance() {
		static Map instance;
		return instance;
	}

	std::vector<map::cross_msg> getCrossings() {
		for (auto it = crossings.cbegin(); it != crossings.cend(); ++it) {
			//int16_t id = (**it).getID();
			//std::vector<int16_t> neighbours = (**it).getNeighbours();
			//std::vector<int16_t> lengths = (**it).getLengths();
	
			//res.crossings.push(*new map::node());
			//**(nodes->end()).neighbours = **it.getNeighbours();
			//**(nodes->end()).lengths = **it.getLengths();
		}

		return *new std::vector<map::cross_msg>();
	}

	int16_t getNumberOfCrossings() {
		return (int16_t)crossings.size();
	}

	static bool getConfig(map::map_config::Request  &req,
         				  map::map_config::Response &res) {

		Map& map = Map::getInstance();

		res.crossings = map.getCrossings();
		res.number_of_crossings = map.getNumberOfCrossings();
	
		ROS_INFO("request : %d", req.req);
		return true;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_config");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("get_map_conifg", Map::getConfig);
	ROS_INFO("Map ready");
	ros::spin();

	return 0;
}
