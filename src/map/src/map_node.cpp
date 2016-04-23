#include "ros/ros.h"
#include <vector>
#include <cstdint>
#include <string>
#include <iostream>
#include <fstream>
#include <cstddef>

#include "map/map_config.h"
#include "map/cross_msg.h"

#include "crossing/crossing.h"

bool getConfig(map::map_config::Request  &req,
         map::map_config::Response &res);

class Map
{
	Map() {}
	Map(const Map&) {}
	
	//shared_ptr
	std::vector<Crossing*> crossings;


public:
	bool configureFromFile(std::string fname)
	{
		//delete &crossings;
		crossings = *new std::vector<Crossing*>();

		std::ifstream conffile;
		//try {
			conffile.open(fname, std::ios::in);
		//}
		if (!conffile.is_open()) return false;

		int16_t rbuf;
		conffile >> rbuf;
		
		struct {
			int16_t ID;
			std::vector<int16_t>* neightsbuf = new std::vector<int16_t>();
			std::vector<int16_t>* lenghtsbuf = new std::vector<int16_t>();
		} crossbuf;

		while (rbuf != 0) {
			crossbuf.ID = rbuf;

			crossbuf.neightsbuf = new std::vector<int16_t>();
			crossbuf.lenghtsbuf = new std::vector<int16_t>();
			
			for (int i = 0; i < 4; ++i) {
				conffile >> rbuf;
				crossbuf.neightsbuf->push_back(rbuf);
			}
			
			for (int i = 0; i < 4; ++i) {
				conffile >> rbuf;
				crossbuf.lenghtsbuf->push_back(rbuf);
			}
			
			crossings.push_back(new Crossing(crossbuf.ID, crossbuf.neightsbuf, crossbuf.lenghtsbuf));

			conffile >> rbuf;
		}

		conffile.close();

		return true;
	}

	static Map& getInstance() {
		static Map instance;
		return instance;
	}

	bool getCrossings(map::map_config::Response &res) {
		for (auto it = crossings.cbegin(); it != crossings.cend(); ++it) {
			map::cross_msg msg;
			msg.ID = (**it).getID();
			std::vector<int16_t>& neighbours = (**it).getNeighbours();
			for (auto it = neighbours.cbegin(); it!= neighbours.cend(); ++it) {
				msg.neighbours.push_back(*it);
			}

			std::vector<int16_t>& lengths = (**it).getLengths();
			for (auto it = lengths.cbegin(); it!= lengths.cend(); ++it) {
				msg.lengths.push_back(*it);
			}

			res.crossings.push_back(msg);
		}

		return true;
	}

	int16_t getNumberOfCrossings() {
		return (int16_t)crossings.size();
	}

	static bool getConfig(map::map_config::Request  &req,
         				  map::map_config::Response &res) {

		Map& map = Map::getInstance();

		map.getCrossings(res);
		res.number_of_crossings = map.getNumberOfCrossings();
	
		ROS_INFO("Map server received request %d", req.req);
		return true;
	}
};

int main(int argc, char **argv)
{
	std::string confname;
	std::cout << "Map config file name is: ";
	std::cin >> confname;


	if (!Map::getInstance().configureFromFile(confname)) { ROS_INFO("Unable to read conf file"); return 1; }

	ros::init(argc, argv, "map_config");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("get_map_config", Map::getConfig);
	ROS_INFO("Map ready");
	ros::spin();

	return 0;
}
