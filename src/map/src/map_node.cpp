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
private:
	int16_t ID;
	std::vector<int16_t>* neighbours;
	std::vector<int16_t>* lengths;

public:
	Crossing(int16_t ID, std::vector<int16_t>* neighbours, std::vector<int16_t>* lengths)
	{
		this->neighbours = neighbours;
		this->lengths = lengths;
		this->ID = ID;
	}

	~Crossing()
	{
		delete neighbours;
		delete lengths;
	}	

	int16_t getID() { return ID; }
	std::vector<int16_t>& getNeighbours() { return *neighbours; }
	std::vector<int16_t>& getLengths() { return *lengths; }
	
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
		auto retvec = *new std::vector<map::cross_msg>();
		for (auto it = crossings.cbegin(); it != crossings.cend(); ++it) {
			int16_t id = (**it).getID();
			std::vector<int16_t>& neighbours = (**it).getNeighbours();
			std::vector<int16_t>& lengths = (**it).getLengths();
	
			retvec.push_back(*new map::cross_msg());
			retvec.end()->neighbours = (**it).getNeighbours();
			retvec.end()->lengths = (**it).getLengths();
		}

		return retvec;
	}

	int16_t getNumberOfCrossings() {
		return (int16_t)crossings.size();
	}

	static bool getConfig(map::map_config::Request  &req,
         				  map::map_config::Response &res) {

		Map& map = Map::getInstance();

		res.crossings = map.getCrossings();
		res.number_of_crossings = map.getNumberOfCrossings();
	
		ROS_INFO("Map server received request %d", req.req);
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
