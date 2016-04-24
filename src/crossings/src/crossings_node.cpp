#include "crossings.h"
#include "map/crossing_init.h"
#include "map/cross_msg.h"
#include "crossings/autocross_msg.h"

void crossCallback(const crossings::autocross_msg::ConstPtr& msg)
{
    if(msg->isMsgFromAuto == true)
    {
        ROS_INFO("I get msg from auto with ID:%d", msg->autoID);
    }
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
    
    ros::init(argc, argv, ss.str().c_str());

    ros::Publisher crossPub = n.advertise<crossings::autocross_msg>(ss.str().c_str(), 1000);
    ros::Subscriber crossSub = n.subscribe(ss.str().c_str(), 1000, crossCallback);

    ros::spin();

    return 0;
}
