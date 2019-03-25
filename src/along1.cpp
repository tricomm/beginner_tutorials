#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h> //小车移动的消息类型
#include<move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include "beginner_tutorials/Num.h"
#include "beginner_tutorials/Result.h"
#include <math.h>
#include <vector>
#include<fstream>
#include<string>

void getMsgFromlist(std::vector<geometry_msgs::Pose>::iterator point,geometry_msgs::PoseStamped& action_goal);

//ros::NodeHandle n;//节点句柄
//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);//准备发布话题 话题类型模板给出,话题名,话题长度参数给出

const double PI=3.14159265359;
double interval = 0.25;

std::vector<geometry_msgs::Pose>::iterator itera_ports;
std::vector <geometry_msgs::Pose> posts; //从文件中读到
ros::Publisher* chatter_pub;
int count = 0;


std::vector<geometry_msgs::Pose> IntervalPoint(geometry_msgs::Pose line_begin,geometry_msgs::Pose line_end){
    std::vector< geometry_msgs::Pose > result;
    geometry_msgs::Pose current_pose;
    current_pose.position.x=line_begin.position.x;
    current_pose.position.y=line_begin.position.y;
    current_pose.orientation=line_begin.orientation;
    double interval_x,interval_y;
    double distance;
    distance=sqrt(pow((line_end.position.x-line_begin.position.x),2)+pow((line_end.position.y-line_begin.position.y),2));
    interval_x=(line_end.position.x-line_begin.position.x)*interval/distance;
    interval_y=(line_end.position.y-line_begin.position.y)*interval/distance;
    while(current_pose.position.x<=line_end.position.x&&current_pose.position.y<=line_end.position.y){
      result.push_back(current_pose);
      current_pose.position.x+=interval_x;
      current_pose.position.y+=interval_y;
      current_pose.orientation=line_begin.orientation;
    }
    if(line_end.position.x!=current_pose.position.x&&line_end.position.y!=current_pose.position.y)
      result.push_back(line_end);
    for(unsigned int i=0;i<result.size();++i){
      std::cout<<result[i].position.x<<","<<result[i].position.y<<std::endl;
    }
    return result;
}

double dgeToRad(double dge)
{

  dge=fmod(dge,360.0);
  if (dge<0)
    dge+=360.0;
  return dge/360.0*2.0*PI;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

void readPosts(std::vector<geometry_msgs::Pose> &posts,const char file_path[100])//获取文件中的坐标存入posts向量
{
  std::ifstream f ;
  f.open(file_path);
  if (f.bad())
  {
    std::cout<<"error 文件出错";
    return;
  } 
  while(!f.eof())
  {
    geometry_msgs::Pose temp_pose;
    double temp_dge;
    char one_world;
    std::string tmp_str;
    f>>temp_pose.position.x;
    f>>temp_pose.position.y;
    f>>temp_dge;
    temp_pose.orientation.x=0;
    temp_pose.orientation.y=0;
    temp_pose.orientation.z=sin(dgeToRad(temp_dge)/2.0);
    temp_pose.orientation.w=cos(dgeToRad(temp_dge)/2.0);
    posts.push_back(temp_pose);
  }
  
}

void chatterCallback( const move_base_msgs::MoveBaseActionResult::ConstPtr& result)
{
  ROS_INFO("I hear:[%s] ", result->status.text.c_str());
  if(itera_ports == posts.end())
  {
    ROS_INFO("reached");
  }
  else if (result->status.text=="Goal reached.")//下一个点
  {
    ROS_INFO("reach goal %d, publish next order",count);
    itera_ports++;
    geometry_msgs::PoseStamped  action_goal;//发送信息
    getMsgFromlist(itera_ports,action_goal);
    chatter_pub->publish(action_goal);
  }
  else if (result->status.text == "Failed to find a valid plan. Even after executing recovery behaviors.") {
    ROS_INFO("There is an obstruction ahead,waiting");
    ros::Rate wait(0.2);
    wait.sleep();
    geometry_msgs::PoseStamped  action_goal;//发送信息
    getMsgFromlist(itera_ports,action_goal);
    chatter_pub->publish(action_goal);
  }
  else if(result->status.text == "Robot is oscillating. Even after executing recovery behaviors.")
  {
    ROS_INFO("error robot is oscillation");
     ros::Rate wait(0.2);
    wait.sleep();
    geometry_msgs::PoseStamped  action_goal;//发送信息
    getMsgFromlist(itera_ports,action_goal);
    chatter_pub->publish(action_goal);
  }
  
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  std::cout<<"111";
}
void getMsgFromlist(std::vector<geometry_msgs::Pose>::iterator point,geometry_msgs::PoseStamped& action_goal)
{
  count++;
  action_goal.header.seq=count;
  action_goal.header.frame_id="map";
  action_goal.header.stamp = ros::Time::now();
  action_goal.pose = *itera_ports;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "along1");//指定节点名称
  readPosts(posts,"posts.txt");
  itera_ports=posts.begin();


  std::vector<geometry_msgs::Pose> interport = IntervalPoint(*itera_ports,*(itera_ports+1));
  std::cout<<(itera_ports+1)->position.x;
  interport.push_back(*itera_ports);
  interport.push_back(*(itera_ports+1));
  std::cout<<"测试"<<std::endl;
  for(auto i : interport)
  {
    std::cout<<i.position.x<<" "<<i.position.y<<" "<<i.orientation.z<<" "<<i.orientation.w<<std::endl;
  }
  // move_base_msgs::MoveBaseActionGoal action_goal;//发送信息
  // geometry_msgs::PoseStamped action_goal;
  
  // ros::NodeHandle n; //节点句柄发送句柄
  // ros::Publisher chatterp =n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  // chatter_pub = &chatterp;

  // ros::Subscriber sub = n.subscribe("/move_base/result", 10, chatterCallback);
  // std::cout<<"接收器设置成功"<<std::endl;

  // ros::Rate loop_rate(0.5);//ros::Rate对象指定自循环频
  // loop_rate.sleep();  //第一次发布前给master通知的时间

  // getMsgFromlist(itera_ports,action_goal);//获得信息
  // chatter_pub->publish(action_goal);
  

  // ros::spin();


  return 0;
}