#include "crossings.h"
#include "map/crossing_init.h"
#include "map/cross_msg.h"
#include "crossings/autocross_msg.h"
#include "lights/LightState.h"
#include "lights/State.h"

map::cross_msg  crossCfg; // this crossing Configuration
ros::Publisher crossPub;
ros::Subscriber crossSub;
ros::Subscriber lightsSub;

bool isLightsPublishing = false;
lights::LightState currLightsCfg ; // current lights config
void lightsCallback(const lights::LightState::ConstPtr& msg)
{
    isLightsPublishing = true;
    ROS_INFO("Get data from Lights.");
    ROS_INFO("\tn=[%d, %d, %d, %d]", msg->n.A, msg->e.A, msg->s.A, msg->w.A);
    ROS_INFO("\te=[%d, %d, %d, %d]", msg->n.B, msg->e.B, msg->s.B, msg->w.B);
    ROS_INFO("\ts=[%d, %d, %d, %d]", msg->n.C, msg->e.C, msg->s.C, msg->w.C);
    ROS_INFO("\tw=[%d, %d, %d, %d]", msg->n.D, msg->e.D, msg->s.D, msg->w.D);
    currLightsCfg = lights::LightState(*msg);
}

bool getAvailsDirections(crossings::autocross_msg &response)
{
    int currCarPos;
    for(currCarPos = 0; currCarPos < 4; currCarPos++)
        if(crossCfg.neighbours[currCarPos] == response.previousCrossID)
            break;

    lights::State dirStates;
    switch(currCarPos)
    {
        case 0:
            dirStates = currLightsCfg.n;
            break;
        case 1:
            dirStates = currLightsCfg.e;
            break;
        case 2:
            dirStates = currLightsCfg.s;
            break;
        case 3:
            dirStates = currLightsCfg.w;
            break;
        case 4:
            return false;
    }
   
    response.availableDirections.resize(4);
    response.availableDirections[0] = dirStates.A;
    response.availableDirections[1] = dirStates.B;
    response.availableDirections[2] = dirStates.C;
    response.availableDirections[3] = dirStates.D;
    return true;
}

int previousCarsID[4] = {0, 0, 0, 0}; 
bool isCrossOccupied = false;
void fromCarCallback(const crossings::autocross_msg::ConstPtr& msg)
{
    if(msg->isMsgFromAuto == true)
    {
        ROS_INFO("I got msg from auto with ID:%d", msg->autoID);

        if(msg->direction == -1) // send him all avail directions
        {
            ROS_INFO("\tI got msg with direction=-1");
            crossings::autocross_msg response = *msg;
            response.isMsgFromAuto = false;
            
            if(!getAvailsDirections(response))
            {
                ROS_INFO("\tBad previous cross ID=%d ! No response...", response.previousCrossID);
                return;
            }

            crossPub.publish(response);       
        }
        else if(msg->direction >= 0 && msg->direction <= 3)
        {
            ROS_INFO("\tI got msg with direction=%d, A car would like to drive!", msg->direction);
            crossings::autocross_msg response = *msg;
            response.isMsgFromAuto = false;

            if(!getAvailsDirections(response))
            {
                ROS_INFO("\tBad previous cross ID=%d ! No response...", response.previousCrossID);
                return;
            }


        }
    }
}

void timeoutCallback(const ros::TimerEvent&)
{
    if(!isLightsPublishing)
    {
        ROS_INFO("Lights topic is not working! Exiting...");
        ros::shutdown(); // Lights don't send any data. We can't continue;
    }
    else
        isLightsPublishing = false;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "crossing_node", ros::init_options::AnonymousName );
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<map::crossing_init>("init_crossing");
    map::crossing_init srv ;
    
    if(!client.call(srv))
    {
        ROS_INFO("There is a problem with service calling");
        return 1;
    }
    
    if(srv.response.crossing.ID == 0)
    {
        ROS_INFO("Map Server didn't accept my request! :(");
        return 1;
    }

    crossCfg = srv.response.crossing;
    std::stringstream ss;
    ss << "crossing_" << crossCfg.ID;
    
    crossPub = n.advertise<crossings::autocross_msg>(ss.str().c_str(), 1000);
    crossSub = n.subscribe(ss.str().c_str(), 1000, fromCarCallback);

    ROS_INFO("I received following data:");
    ROS_INFO("\tMy ID is %d. My topic name is: %s", crossCfg.ID, ss.str().c_str());
    ROS_INFO("\tNeighbours: n=%d, e=%d, s=%d, w=%d", 
            crossCfg.neighbours[0],crossCfg.neighbours[1],crossCfg.neighbours[2],crossCfg.neighbours[3]);
    ROS_INFO("\tLengths:    n=%d, e=%d, s=%d, w=%d",
            crossCfg.lengths[0], crossCfg.lengths[1], crossCfg.lengths[2], crossCfg.lengths[3]);

    ss.str("");
    ss << "lights_" << crossCfg.ID;

    lightsSub = n.subscribe(ss.str().c_str(), 1000, lightsCallback);
    ros::Timer timer = n.createTimer(ros::Duration(10), timeoutCallback);
    ros::spin();
    return 0;
}
