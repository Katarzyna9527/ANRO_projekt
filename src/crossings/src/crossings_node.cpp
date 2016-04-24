#include "crossings.h"
#include "map/crossing_init.h"
#include "map/cross_msg.h"
#include "crossings/autocross_msg.h"
#include "lights/LightState.h"
#include "lights/State.h"

void crossCallback(const crossings::autocross_msg::ConstPtr& msg)
{
    if(msg->isMsgFromAuto == true)
    {
        ROS_INFO("I get msg from auto with ID:%d", msg->autoID);
    }
}

volatile bool isLightsPublishing = false;
void lightsCallback(const lights::LightState::ConstPtr& msg)
{
    isLightsPublishing = true;
    ROS_INFO("Get data from Lights.");
    ROS_INFO("n=[%d, %d, %d, %d]", msg->n.A, msg->n.B, msg->n.C, msg->n.D);
    ROS_INFO("e=[%d, %d, %d, %d]", msg->e.A, msg->e.B, msg->e.C, msg->e.D);
    ROS_INFO("s=[%d, %d, %d, %d]", msg->s.A, msg->s.B, msg->s.C, msg->s.D);
    ROS_INFO("w=[%d, %d, %d, %d]", msg->w.A, msg->w.B, msg->w.C, msg->w.D);
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
    
    map::cross_msg crossData;

    if(client.call(srv))
    {
        ROS_INFO("Map Server accepted my request. My ID : %d", srv.response.crossing.ID);
        if(srv.response.crossing.ID == 0)
        {
            ROS_INFO("Map Server didn't accept my request! :(");
            return 1;
        }
    }
    else
    {
        ROS_INFO("There is a problem with service calling");
        return 1;
    }

    crossData = srv.response.crossing;
    std::stringstream ss;
    ss << "crossing_" << crossData.ID;
    
    ros::Publisher crossPub = n.advertise<crossings::autocross_msg>(ss.str().c_str(), 1000);
    ros::Subscriber crossSub = n.subscribe(ss.str().c_str(), 1000, crossCallback);

    ROS_INFO("I received following data:");
    ROS_INFO("Neighbours: n=%d, e=%d, s=%d, w=%d", 
            crossData.neighbours[0],crossData.neighbours[1],crossData.neighbours[2],crossData.neighbours[3]);
    ROS_INFO("Lengths:    n=%d, e=%d, s=%d, w=%d",
            crossData.lengths[0], crossData.lengths[1], crossData.lengths[2], crossData.lengths[3]);

    ss.str("");
    ss << "lights_" << crossData.ID;

    ros::Subscriber lightsSub = n.subscribe(ss.str().c_str(), 1000, lightsCallback);
    ros::Timer timer = n.createTimer(ros::Duration(10), timeoutCallback);

    ros::spin();

    return 0;
}
