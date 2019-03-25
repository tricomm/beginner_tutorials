#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"//自定义的srv生成的头文件

bool add(beginner_tutorials::AddTwoInts::Request  &req, //提供ros服务从Request读入,结果装入Response中
         beginner_tutorials::AddTwoInts::Response &res) //这些数据结构都在AddTwoInts.h中自动生成
{                                                       
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);//服务名为add_two_ints
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}