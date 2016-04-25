#include "ros/ros.h"
#include "crossings/autocross_msg.h"
#include <iostream>

ros::Publisher testPub;
ros::Subscriber testSub;
int id;
int stage = 0;
int prev;
int crossId;

void testCallback(const crossings::autocross_msg::ConstPtr& msg)
{
    ros::NodeHandle n;
    ROS_INFO("Get data from crossing");

    if(msg->isMsgFromAuto)
        return;

    if(msg->autoID != id)
        return;

    if(msg->direction == -1) // we receive avails dirs
    {
        ROS_INFO("I receive available directions");
        std::vector<int16_t> avDirs = msg->availableDirections;       
        ROS_INFO("That is: [%d, %d, %d, %d]", avDirs[0], avDirs[1], avDirs[2], avDirs[3]);
        ROS_INFO("The last car on my road is : %d", msg->previousAutoID);

        int destDir;
        std::cout << "Enter direction to drive to: ";
        std::cin >> destDir;

        crossings::autocross_msg response = *msg;
        response.isMsgFromAuto = true;
        response.direction = destDir;

        testPub.publish(response);        
    } else {
        ROS_INFO("I receive acceptation from crossing");
        ROS_INFO("Received data about next cross. ID=%d, len=%d", msg->nextCrossID, msg->length);
        
        std::string str;
        std::cout << "Type enter to leave a crossing";
        std::cin >> str;

        crossings::autocross_msg response = *msg;
        response.isMsgFromAuto = true;
        response.isCrossed = true;
        testPub.publish(response);

        std::cout << "Type enter to drive to next crossing";
        std::cin >> str;

        response.isCrossed = false;
        response.direction = -1;
        response.previousCrossID = crossId;
        
        prev = crossId;
        crossId = msg->nextCrossID;
        
        std::stringstream ss;
        ss << "crossing_" << msg->nextCrossID;
        testPub = n.advertise<crossings::autocross_msg>(ss.str().c_str(), 1000, true);
        testSub = n.subscribe(ss.str().c_str(), 1000, testCallback);
        
        testPub.publish(response);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crossings_test_node");
    ros::NodeHandle n;
    
    std::cout << "Type cross ID to drive : ";
    std::cin >> crossId;

    std::cout << "Type this node ID: ";
    std::cin >> id;

    std::cout << "Type previous cross ID: ";
    std::cin >> prev;

    std::stringstream ss;
    ss << "crossing_" << crossId;

    testPub = n.advertise<crossings::autocross_msg>(ss.str().c_str(), 1000, true);
    testSub = n.subscribe(ss.str().c_str(), 1000, testCallback);
    
    crossings::autocross_msg msg;
    msg.autoID = id;
    msg.isMsgFromAuto = true;
    msg.direction = -1;
    msg.isCrossed = false;
    msg.previousCrossID = prev;
    testPub.publish(msg);

    ROS_INFO("Send data to topic %s with direction=-1.", ss.str().c_str()); 

    ros::spin();
    return 0;
}
