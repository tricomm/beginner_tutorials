#include <ros/ros.h>
#include <tf/transform_listener.h>//为简化接收任务
#include <geometry_msgs/PoseStamped.h>
#include <turtlesim/Spawn.h>

int count = 0;
int main(int argc, char** argv){
  ros::init(argc, argv, "pro_tf_listener");

  ros::NodeHandle node;

  ros::Publisher turtle_vel =
  node.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);

  tf::TransformListener listener;//监听器变量需要缓冲10秒左右,所以要让它持久化
  ros::Rate rate(10.0);

  while (node.ok()){
    tf::StampedTransform transform;
    //ros::Duration(1.0).sleep();
    
    

  try{
    listener.lookupTransform("/map","/base_link", ros::Time(0), transform);
  //1.参考坐标系,2.参考系时间,
  }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id = count;
    robot_pose.header.frame_id = "map";
    robot_pose.header.stamp = ros::Time::now();
    robot_pose.pose.position.x = transform.getOrigin().x();
    robot_pose.pose.position.y = transform.getOrigin().y();
    robot_pose.pose.position.z = transform.getOrigin().z();
    robot_pose.pose.orientation.w = transform.getRotation().w();
    robot_pose.pose.orientation.x = transform.getRotation().x();
    robot_pose.pose.orientation.y = transform.getRotation().y();
    robot_pose.pose.orientation.z = transform.getRotation().z();
    count++;
    
    turtle_vel.publish(robot_pose);

    rate.sleep();
  }
  return 0;
};
