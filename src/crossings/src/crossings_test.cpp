#include "ros/ros.h"
#include "crossings/autocross_msg.h"
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crossings_test_node");
    ros::NodeHandle n;
    
    std::string str ;
    std::cout << "Type topic to publish to: ";
    std::cin >> str;

    int id;
    std::cout << "Type this node ID: ";
    std::cin >> id;

    ros::Publisher testPub = n.advertise<crossings::autocross_msg>(str.c_str(), 1000, true);
    ros::Rate loop_rate(1);

    short int count = 0;
    while(ros::ok())
    {
        crossings::autocross_msg msg;
        msg.autoID = id;
        msg.isMsgFromAuto = true;

        testPub.publish(msg);

        ROS_INFO("Send data to topic %s. Count = %d", str.c_str(), count); 

        ros::spinOnce();
        loop_rate.sleep();
        ++count ;
    }
 

    return 0;
}
