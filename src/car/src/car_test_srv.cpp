#include "ros/ros.h"
#include <vector>
#include <cstdint>
#include <string>
#include <iostream>
#include <fstream>
#include <cstddef>

#include "car/car_init.h"

bool fillService(car::car_init::Request  &req,
         				  car::car_init::Response &res)
{
  res.carID=1;
  res.pathLenght=100;
  res.prevCrossing=4;
  res.nextCrossin=5;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;
  

  ros::ServiceServer service = n.advertiseService("car_ini", fillService);
  ros::spin();

  return 0;
