#include "crossings_lib.h"
#include <signal.h>

void killTaskHandler(int sig)
{
    ROS_INFO("crossing_node: Going to shutdown...");

    Crossing::getInstance().shutdownAll();
    ros::shutdown();
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "crossing_node", ros::init_options::AnonymousName );
    ros::NodeHandle n;

    if(!Crossing::getInstance().initCrossing())
    {
        ROS_INFO("crossing_node: I have to exit...");
        return 1;
    }

    signal(SIGINT, killTaskHandler);
    ros::spin();
    
    return 0;
}
