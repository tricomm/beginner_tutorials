#include "ros/ros.h"
#include "std_msgs/String.h"

//回调函数,参数为话题中的消息
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
 //任何ROSnode都要先初始化
  ros::init(argc, argv, "listener");

  //定义Node句柄,会完全初始化此节点
  ros::NodeHandle n;


 //告诉mastet node 想接收消息的方式,如下为接收chatter主题,收到是调用chatterCallback回调函数,缓存队列1000
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  
  //进入循环 ctrl-C时退出或被master node关闭
  ros::spin();

  return 0;
}
