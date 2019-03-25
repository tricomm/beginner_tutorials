#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include<sensor_msgs/LaserScan.h>
#include <math.h>
#include <vector>
#include<vector>

using namespace std;
double dgeToRad(double dge)
{
  dge=fmod(dge,360.0);
  if (dge<0)
    dge+=360.0;
  return dge/360.0*2.0*PI;
}

double point2PointDistance(const geometry_msgs::Pose& Apose,const geometry_msgs::Pose& Bpose)
{
    double dx = Apose.position.x - Bpose.position.x;
    double dy = Apose.position.y - Bpose.position.y;
    return sqrt(dx*dx+dy*dy);
}

double point2LineDistance(const geometry_msgs::Pose& current_pose,const geometry_msgs::Pose& line_begin,const geometry_msgs::Pose& line_end){
  double A,B,C,D,distance;
  A=-line_begin.position.y+line_end.position.y;
  B=line_begin.position.x-line_end.position.x;
  C=line_begin.position.y*line_end.position.x-line_begin.position.x*line_end.position.y;
  D=sqrt(A*A+B*B);
  std::cout<<A<<","<<B<<","<<C<<","<<D<<std::endl;
  distance=abs(A*current_pose.position.x+B*current_pose.position.y+C)/D;
  return distance;
}

//拆分线段
vector<geometry_msgs::Pose> IntervalPoint(geometry_msgs::Pose line_begin,geometry_msgs::Pose line_end,double interval){
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