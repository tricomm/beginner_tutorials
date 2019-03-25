#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)//argc-1为参数个数
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");//为add_two_ints服务创建client
  beginner_tutorials::AddTwoInts srv;//创建服务数据类型
  srv.request.a = atoll(argv[1]);//写入request信息
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))//调用成功则输出response信息
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}