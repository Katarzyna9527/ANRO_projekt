#include "crossings_lib.h"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "crossing_node", ros::init_options::AnonymousName );

    if(!Crossing::getInstance().initCrossing())
    {
        ROS_INFO("crossing_node: I have to exit...");
        return 1;
    }

    ros::spin();
    
    return 0;
}
