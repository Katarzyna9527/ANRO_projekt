#include "ros/ros.h"
#include "car/car_init.h"

bool fillMessage(car::car_init::Request  &req,
         car::car_init::Response &res){
  if(req.req = 0){
    res.carID=1;
    res.pathLenght=100;
    res.nextCrossing=1;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_init_test");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("init_car", fillMessage);
  ROS_INFO("Ready to init the car.");
  ros::spin();

  return 0;
}
