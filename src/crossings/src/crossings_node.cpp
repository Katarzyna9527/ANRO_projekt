#include "ros/ros.h"
#include "crossings/add_crossing.h"
#include "crossings/cross_msg.h"

class Crossing
{
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

class CrossSrvr // Crossings Server
{

    CrossSrvr() {}
    CrossSrvr(const CrossSrvr&) {}

    std::vector<Crossing*> crossings;
    
    bool checkIfExistID(int16_t ID)
    {
        for(auto it = crossings.cbegin(); it != crossings.cend(); ++it)
            if((*it)->getID() == ID)
                return true;
        return false;
    }

public:
    void addCrossing(Crossing* newCross)
    {
        crossings.push_back(newCross);
        // create new topic for cars
    }

    static CrossSrvr& getInstance()
    {
        static CrossSrvr thisInstance;
        return thisInstance;
    }

    static bool addCross(crossings::add_crossing::Request  &req,
         crossings::add_crossing::Response &res)
    {
	    // place some code to get data from MAP about new CROSSING
        CrossSrvr& crossSrvr = CrossSrvr::getInstance();
        int16_t tmpID;

    	for(int i = 0 ; i < req.crossingsData.size() ; i++)
    	{
            tmpID = req.crossingsData[i].ID;        
    
            if(!crossSrvr.checkIfExistID(tmpID))
            {
                crossSrvr.addCrossing(new Crossing(
                            tmpID,
                            new std::vector<int16_t>(req.crossingsData[i].neighbours),
                            new std::vector<int16_t>(req.crossingsData[i].lengths)));
                ROS_INFO("Added new crossing. ID: %d. It\'s talking on \'cross_%d \' topic.", tmpID, tmpID);
    	    }
            else
            {
                ROS_INFO("Requested crossing with ID: %d arleady exist!", tmpID);
            }
        }
      return true;
    }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "crossings_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_crossing_service", CrossSrvr::addCross);
  ROS_INFO("Ready to add new crossings.");
 
  ros::spin();

 return 0;
}
