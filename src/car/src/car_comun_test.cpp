#include "ros/ros.h"
#include "car/autocross_msg.h"
#include <iostream>
ros::Publisher crossPub;
ros::Subscriber crossSub;
bool isMsgFromAuto=false;
int16_t autoID=0;  
int16_t previousCrossID=0;
int16_t direction=0;
bool isCrossed=false;
int16_t availableDirections[4];
int16_t length=0;
int16_t nextCrossID=0;
int16_t previousAutoID=0 ;
car::autocross_msg msgA;
void testCallback(const car::autocross_msg::ConstPtr& msg)
{
    if(msg->isMsgFromAuto){
      ROS_INFO("got a message");
      if(msg->direction == -1){
         msgA.previousAutoID=0;  
         int16_t myints[] = {1,1,1,1};
         std::vector<int16_t> avDirs (myints, myints + sizeof(myints) / sizeof(int) );
         msgA.availableDirections=avDirs;
         crossPub.publish(msgA);
         ROS_INFO("has sent previous auto id");
      }
      else{
         msgA.nextCrossID=5;
         crossPub.publish(msgA);
      }
    }
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "crossing_test", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ROS_INFO("ready to iniciate connection");
    crossPub = n.advertise<car::autocross_msg>("crossing_1", 1000);//crossing
    //ROS_INFO("1");
    crossSub = n.subscribe("crossing_1", 1000, testCallback);
    //ROS_INFO("2");
    ros::spin();
    return 0;
}
