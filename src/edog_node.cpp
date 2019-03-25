#define PI 3.14159265359
#define ZERO 0.2
#define STEP 0.5

#include"distance.h"
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h> //小车移动的消息类型
#include<move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include<sensor_msgs/LaserScan.h>
#include <math.h>
#include <vector>
#include<queue>
#include<fstream>
#include<string>
#include<sqlite3.h>

double interval = 0.25;
geometry_msgs::Pose current_pose;
geometry_msgs::Pose current_goal;
geometry_msgs::Pose* current_goal_ptr=&current_goal;
sensor_msgs::LaserScan current_scan;
std::vector<geometry_msgs::Pose>::iterator itera_posts;//当前出发点T
std::vector<geometry_msgs::Pose>::iterator itera_globle_posts;//当前出发点T
std::vector <geometry_msgs::Pose> posts; //从文件中读到
std::queue <geometry_msgs::Pose> globle_posts; //从文件中读到
bool localbegin = false;
bool globlebegin =false;
bool path_correction = false;
ros::Publisher* chatter_pub;
int cnt = 0;
bool find_obstacle=false;
geometry_msgs::PoseStamped action_goal;
void readPose(std::string FilePath,std::queue< geometry_msgs::Pose > &Poses_){
/* read object poses from FilePath, and save it in vector Poses*/
/* const char *SelectSentence = "SELECT * from global_goal order by ID ASC"; 
  sqlite3_stmt *stmt = NULL;

    //进行查询前的准备工作——检查语句合法性
    //-1代表系统会自动计算SQL语句的长度
  int result = sqlite3_prepare_v2(sql, sqlSentence, -1, &stmt, NULL);
  if (result == SQLITE_OK) {
    std::clog <<  "查询语句OK";
    // 每调一次sqlite3_step()函数，stmt语句句柄就会指向下一条记录
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      // 取出第0列字段的值
      const unsigned char *name = sqlite3_column_text(stmt, 0);
      // 取出第1列字段的值
      int age = sqlite3_column_int(stmt, 1);
      //输出相关查询的数据
      std::clog << "name = " << name <<", age = "<< age;
      }
  }else {
        std::clog << "查询语句有问题";
  }
    //清理语句句柄，准备执行下一个语句
  sqlite3_finalize(stmt);*/
  std::ifstream f(FilePath);
  if(f.is_open()){
    ROS_INFO("Loading poses from path: ");
    while(!f.eof()){
      geometry_msgs::Pose pose;
      f>>pose.position.x;
      f>>pose.position.y;
      f>>pose.orientation.z;
      f>>pose.orientation.w;
      Poses_.push(pose);
    }
  }else{
    ROS_ERROR("Open file failed, check the path:");
  }
}

void scanCallback(const sensor_msgs::LaserScanConstPtr scan){
  ROS_INFO("scanCallback");
  action_goal.pose.position.x=current_goal_ptr->position.x+current_pose.position.x;
  action_goal.pose.position.y=current_goal_ptr->position.y+current_pose.position.y;
  action_goal.pose.orientation=current_goal_ptr->orientation;
  if(current_goal_ptr->position.x==0)current_goal_ptr->position.x+=0.00001;
  double theta=atan(current_goal_ptr->position.y/current_goal_ptr->position.x);
  if(current_goal_ptr->position.x<0){
    theta=-theta;
  }
  find_obstacle=false;
  for(unsigned int i=0;i<scan->ranges.size();++i){
    double delta_theta=abs(scan->angle_min+i*scan->angle_increment-theta);
    if(delta_theta<0.1&&scan->ranges[i]<STEP){
      find_obstacle=true;
      ROS_INFO("Find obstacle, waiting...");
      break;
    }
  }
}
void chatterCallback( const move_base_msgs::MoveBaseActionResult::ConstPtr& result)
{ 
  if (result->status.text == "Robot is oscillating. Even after executing recovery behaviors.") 
  {
    std::cout<<"error 机器人出现obstruction错误"<<std::endl;
  }
}

void postCallback( const geometry_msgs::PoseStamped::ConstPtr& posestamped )
{
    current_pose=posestamped->pose;
    while(!globle_posts.empty()){
      if(point2PointDistance(globle_posts.front(),current_pose)<STEP){
        globle_posts.pop();
      }else{
        current_goal_ptr->position.x=(globle_posts.front().position.x-current_pose.position.x)*STEP/point2PointDistance(globle_posts.front(),current_pose);
        current_goal_ptr->position.y=(globle_posts.front().position.y-current_pose.position.y)*STEP/point2PointDistance(globle_posts.front(),current_pose);
        current_goal_ptr->orientation.w=(current_pose.orientation.w+current_goal_ptr->orientation.w)/2;
        current_goal_ptr->orientation.z=(current_pose.orientation.z+current_goal_ptr->orientation.z)/2;
        break;
      }
    }
}

int main(int argc, char **argv)
{
  sqlite3* db;
  int rc;
  rc=sqlite3_open("edog.db",&db);
  char *zErrMsg = 0;
  if( rc ){
      fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
      exit(0);
   }else{
      fprintf(stderr, "Opened database successfully\n");
   }
  sqlite3_close(db);
  ros::init(argc, argv, "control");
  readPose("posts.txt",globle_posts);
  action_goal.pose.position.x=0;
  action_goal.pose.position.y=1;
  ros::NodeHandle n;
  
  ros::Publisher chatterp =n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
  ros::Rate loop_rate(1);
  chatter_pub = &chatterp;
  chatterp.publish(action_goal);
  loop_rate.sleep();

  ros::Subscriber sub_result = n.subscribe("/move_base/result", 10, chatterCallback);
  ros::Subscriber sub_pose = n.subscribe("/robot_pose",10, postCallback);
  ros::Subscriber sub_scan=n.subscribe("/scan",10,scanCallback);
  while(ros::ok()){
    if(!find_obstacle){
      cnt++;
      action_goal.header.seq=cnt;
      action_goal.header.frame_id="map";
      action_goal.header.stamp = ros::Time::now();
      std::cout<<"BEFORE PUBLISH:ACTION_GAOL"<<action_goal.pose.orientation.w<<","<<action_goal.pose.orientation.z<<std::endl;
      chatter_pub->publish(action_goal);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}