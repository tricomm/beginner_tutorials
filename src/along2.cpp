#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h> //小车移动的消息类型
#include<move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include<sensor_msgs/LaserScan.h> //雷达数据类型
#include "beginner_tutorials/Result.h"


#include<tf/transform_broadcaster.h>
#include <math.h>
#include <vector>
#include<fstream>
#include<string>

void getMsgFromlist(geometry_msgs::Pose point,geometry_msgs::PoseStamped& action_goal);

void getMsgFromlist(std::vector<geometry_msgs::Pose>::iterator point,geometry_msgs::PoseStamped& action_goal);

//ros::NodeHandle n;//节点句柄
//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);//准备发布话题 话题类型模板给出,话题名,话题长度参数给出

const double PI=3.14159265359;
const double ZERO = 0.2;
const double interval = 0.25;//步长 两步之内是障碍物
const double TIME_OUT = 3;
const int obstac_min = 20;
double FX = 0;//pro为1 2.5为0
std::vector<geometry_msgs::Pose>::iterator itera_posts;//当前出发点T
std::vector<geometry_msgs::Pose>::iterator itera_globle_posts;//当前出发点T
std::vector <geometry_msgs::Pose> posts; //从文件中读到
std::vector <geometry_msgs::Pose> globle_posts; //从文件中读到
bool localbegin = false;
bool globlebegin =false;
bool path_correction = false;
ros::Publisher* chatter_pub;
int count = 0;

geometry_msgs::PoseStamped  action_goal;//发送信息
geometry_msgs::PoseStamped  now_pose;//实时位置
bool find_obstacle = false;
bool wait_distory_obstacle = false;


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
double point2PointYaw(const geometry_msgs::Pose& Apose,const geometry_msgs::Pose& Bpose)
{
  double Aroll,Apitch,Ayaw,Broll,Bpitch,Byaw;
  tf::Matrix3x3(tf::Quaternion(Apose.orientation.x,Apose.orientation.y
  ,Apose.orientation.z,Apose.orientation.w)).getRPY(Aroll,Apitch,Ayaw);
  tf::Matrix3x3(tf::Quaternion(Bpose.orientation.x,Bpose.orientation.y
  ,Bpose.orientation.z,Bpose.orientation.w)).getRPY(Broll,Bpitch,Byaw);
  //std::cout<<"AYaw: "<<Ayaw<<"Byaw: "<<Byaw<<std::endl;
  double Dyaw = fabs(Ayaw - Byaw);
  Dyaw = (2*PI-Dyaw)<Dyaw ? (2*PI - Dyaw):Dyaw; 
  return Dyaw;
}

double point2LineDistance(const geometry_msgs::Pose& current_pose,const geometry_msgs::Pose& line_begin,const geometry_msgs::Pose& line_end){
  double A,B,C,D,distance;
  A=-line_begin.position.y+line_end.position.y;
  B=line_begin.position.x-line_end.position.x;
  C=line_begin.position.y*line_end.position.x-line_begin.position.x*line_end.position.y;
  D=sqrt(A*A+B*B);
  //std::cout<<A<<","<<B<<","<<C<<","<<D<<std::endl;
  distance=abs(A*current_pose.position.x+B*current_pose.position.y+C)/D;
  return distance;
}

inline void gotoLocalTPlusI(int i)
{
  getMsgFromlist(itera_posts+i,action_goal);
  ros::Duration(0.1).sleep();//查看有没有障碍物
  if(!find_obstacle)
  {
    chatter_pub->publish(action_goal);
    count++;
        //ROS_INFO("reach goal %d, publish next order",count)
    std::cout<<"到达全局目标"<<itera_globle_posts-globle_posts.begin()<<"下的"
            <<"局部目标:"<<itera_posts-posts.begin()+1<<"局部目标:"<<itera_posts-posts.begin()+1+i
            <<"发送成功"<<std::endl;
    std::cout<<"监听到当前位置x "<<now_pose.pose.position.x
            <<"y "<<now_pose.pose.position.y<<std::endl;
  }
}

//拆分线段2.0
std::vector<geometry_msgs::Pose> IntervalPoint(geometry_msgs::Pose line_begin,geometry_msgs::Pose line_end){
   std::cout<<"输入"<<line_begin.position.x<<' '<<line_begin.position.y<<"到"
     <<line_end.position.x<<' '<<line_end.position.y<<"拆分结果为:"<<std::endl;

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
    while(point2PointDistance(current_pose,line_end)>interval){//STEP是最小距离
      result.push_back(current_pose);
      current_pose.position.x+=interval_x;
      current_pose.position.y+=interval_y;
      current_pose.orientation=line_begin.orientation;
    }
    result.push_back(current_pose);//修改1

    if(line_end.position.x!=current_pose.position.x||line_end.position.y!=current_pose.position.y)//修改2
      result.push_back(line_end);

    
    for(unsigned int i=0;i<result.size();++i){
      std::cout<<result[i].position.x<<","<<result[i].position.y<<std::endl;
    }
    return result;
}

void readPosts(std::vector<geometry_msgs::Pose> &posts,const char file_path[100])//获取文件中的坐标存入posts向量 多加一个0 0 0
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
  std::cout<<"读取posts.txt成功"<<std::endl;
}

//监听雷达信息进行坐标转换和障碍物发现
void scanCallback(const sensor_msgs::LaserScanConstPtr scan){

  //ROS_INFO("scanCallback");
  double roll,pitch,yaw;//雷达方向
  tf::Matrix3x3(tf::Quaternion(now_pose.pose.orientation.x,now_pose.pose.orientation.y
  ,now_pose.pose.orientation.z,now_pose.pose.orientation.w)).getRPY(roll,pitch,yaw);
  //std::cout<<"当前角度: "<<roll<<' '<<pitch<<' '<<yaw<<std::endl;
  double theta1 = atan2(action_goal.pose.position.y-now_pose.pose.position.y,
                  action_goal.pose.position.x-now_pose.pose.position.x);
  theta1 = std::isnan(theta1) ? 0:theta1;//世界坐标中目标相对雷达的方向
  yaw = std::isnan(yaw)?0:yaw;
  double theta2 = theta1-yaw;//目标在雷达坐标中的角度  //yaw:雷达相对世界坐标的朝向差
  if(theta2>PI)theta2-=2*PI;
  else if(theta2<-PI)theta2+=2*PI;
  std::cout<<"yaw:"<<yaw<<" theta1"<<theta1<<" theta2:"<<theta2<<std::endl;
  theta2 = theta2/PI*180;
  int Pranges=180-theta2-30-FX*90;//自身与目标连线正负30度 雷达数据开始的数组下表
  int obstac_num=0;
  std::cout<<"Ptemp:"<<Pranges<<std::endl;
  for(int i=0;i<60;++i)
  {
    int Ptemp = (Pranges+i)%360;
    if(scan->ranges[Ptemp]<2*interval)//当前与目标距离
      {
        obstac_num++;
        std::cout<<scan->ranges[Ptemp];
      }
  }
  if(obstac_num>obstac_min)
    find_obstacle=true;
  else
    find_obstacle=false;
  
  std::cout<<"前方障碍物数量"<<obstac_num<<std::endl;
}
//监听机器人回复信息,报告机器人错误
void chatterCallback( const move_base_msgs::MoveBaseActionResult::ConstPtr& result)
{
  
  if (result->status.text == "Robot is oscillating. Even after executing recovery behaviors.") 
  {
   // ROS_INFO("There is an obstruction ahead,waiting");
    std::cout<<"error 机器人出现obstruction错误"<<std::endl;
  }

  // else if (result->status.text == "Failed to find a valid plan. Even after executing recovery behaviors.") 
  // {
  //  // ROS_INFO("There is an obstruction ahead,waiting");
  //   geometry_msgs::Pose goal_point;
  //   if(itera_posts+2<posts.end())
  //   {
  //     goal_point = *(itera_posts+2);
  //   }
  //   else if(itera_posts+2==posts.end())
  //   {
  //     goal_point = *(itera_posts+1);
  //   }
  //   else
  //   {
  //     std::cout<<"error 路径规划失败处理部分出意料外错误";
  //   }
  //   ros::Rate rate(0.25);
  //   rate.sleep();
  //   getMsgFromlist(goal_point,action_goal);
  //   chatter_pub->publish(action_goal);   
      
  // }
  

}

//监听机器人姿态pose信息,核心控制
void postCallback( const geometry_msgs::PoseStamped::ConstPtr& posestamped )
{
  static auto no_move_time = ros::Time::now();

  //重发机制
  if(now_pose.pose.position.x - posestamped->pose.position.x > 0.1*ZERO
  ||now_pose.pose.position.y - posestamped->pose.position.y > 0.1*ZERO
  ||now_pose.pose.position.z - posestamped->pose.position.z  > 0.1*ZERO
  || now_pose.pose.orientation.w - posestamped->pose.orientation.w > 0.1*ZERO
  || now_pose.pose.orientation.x - posestamped->pose.orientation.x > 0.1*ZERO
  || now_pose.pose.orientation.y - posestamped->pose.orientation.y > 0.1*ZERO
  || now_pose.pose.orientation.z - posestamped->pose.orientation.z > 0.1*ZERO)
  {
    //std::cout<<"正在移动"<<std::endl;
    now_pose = *posestamped;
    no_move_time = ros::Time::now();
  }
  else if((ros::Time::now().toSec() - no_move_time.toSec()) > 5.0)
  {
    chatter_pub->publish(action_goal);
    no_move_time = ros::Time::now();
    std::cout<<"三秒以上未移动当前位置"<<now_pose.pose.position.x<<" "<<now_pose.pose.position.y
            <<"重发"<<action_goal.pose.position.x<<" "<<action_goal.pose.position.y
            <<"距离:"<<point2PointDistance(now_pose.pose,action_goal.pose)
            <<"角度:"<<point2PointYaw(now_pose.pose,action_goal.pose)
            <<"在雷达坐标系中的角度"<<atan2(action_goal.pose.position.y-now_pose.pose.position.y,
            action_goal.pose.position.x-now_pose.pose.position.x)<<std::endl;
            //atan2 对(第一参数)/临(第二参数)
  }
  
  //  std::cout<<"监听到当前位置x "<<posestamped->pose.position.x
  //              <<"y "<<posestamped->pose.position.y<<std::endl;
  if(find_obstacle)
  {
    chatter_pub->publish(now_pose);
    std::cout<<"发现障碍物等待移除"<<std::endl;
    ros::Duration(0.5).sleep();
    wait_distory_obstacle = true;
  }
  else if(!find_obstacle&&wait_distory_obstacle)
  {
    std::cout<<"障碍物已移除"<<std::endl;
    chatter_pub->publish(action_goal);
    wait_distory_obstacle=false;
  }
  // else
  // {
  //   chatter_pub->publish(action_goal);
  //   std::cout<<action_goal.pose.position.x<<' '<<action_goal.pose.position.y<<std::endl;
  // }
  

  double globle_goal_distance = point2PointDistance(*itera_globle_posts,posestamped->pose);
  double globle_goal_yaw = point2PointYaw(*itera_globle_posts,posestamped->pose);

    //全局规划
  if( itera_globle_posts == globle_posts.end()) //当前点到了则++,加到了尾后则完成
  {
      std::cout<<"到达全局最终目标"<<std::endl;
      std::cout<<"监听到当前位置x "<<posestamped->pose.position.x
              <<"y "<<posestamped->pose.position.y<<std::endl;
      return;
  }

  else if(globle_goal_distance < ZERO && globle_goal_yaw < ZERO*5)
  {
      globlebegin = true;
      std::cout<<"到达全局目标:"<<itera_globle_posts-globle_posts.begin()+1<<std::endl;
      posts = IntervalPoint(*itera_globle_posts,*(itera_globle_posts+1));
      //posts = globle_posts; //debug
      itera_posts = posts.begin();
      itera_globle_posts++;

      std::cout<<"到达全局目标"<<itera_globle_posts-globle_posts.begin()<<"下的"
                        <<"局部目标:"<<itera_posts-posts.begin()+1<<std::endl;
              std::cout<<"监听到当前位置x "<<posestamped->pose.position.x
              <<"y "<<posestamped->pose.position.y<<std::endl;
  }
    
    //局部规划
  if(globlebegin)
  {
    //启动器
    if(!localbegin 
    && itera_posts == posts.begin() 
    && point2PointDistance(*itera_posts,posestamped->pose)<ZERO 
    && point2PointYaw(*itera_posts,posestamped->pose)<ZERO*5) //1局部未启动 2局部目标已更新
    {
        getMsgFromlist(itera_posts+2,action_goal);
        ros::Duration(0.1).sleep();//查看有没有障碍物
        if(!find_obstacle)
        {
          chatter_pub->publish(action_goal);
          localbegin = true;
        }
    }

    double goal_distance =  point2PointDistance(*(itera_posts+1),posestamped -> pose);
    double goal_angle = point2PointYaw(*(itera_posts+1),posestamped -> pose);
    if(goal_distance < ZERO && goal_angle < ZERO*5)//到了T+1
    {
      localbegin = false;
      std::cout<<"到达T+1"<<std::endl;
      itera_posts +=1;        //到了T+1则++ 等价于令T点=T+1
      if(itera_posts+2 < posts.end())
      {
        gotoLocalTPlusI(2);
      }
      else if(itera_posts+2 == posts.end())
      {
            count++;
            //ROS_INFO("reach goal %d, publish next order",count);

            std::cout<<"zzz到达全局目标"<<itera_globle_posts-globle_posts.begin()<<"下的"
                      <<"局部目标:"<<itera_posts-posts.begin()+1<<std::endl;
            std::cout<<"监听到当前位置x "<<posestamped->pose.position.x
            <<"y "<<posestamped->pose.position.y<<std::endl;

      }
      else if(itera_posts+1 == posts.end())//用不到
      {
                //ROS_INFO("reached");
            std::cout<<"到达全局目标"<<itera_globle_posts-globle_posts.begin()<<"下的"
                      <<"局部总目标"<<std::endl;
            std::cout<<"监听到当前位置x "<<posestamped->pose.position.x
            <<"y "<<posestamped->pose.position.y<<std::endl;
      }
    }
    else
    {
      if(itera_posts+3<posts.end())//未经过T+1,到达了T+2
      { 
       //std::cout<<"跳过了纠正啊"<<"到达全局目标"<<itera_globle_posts-globle_posts.begin()<<"下的"
         //   <<"局部目标:"<<itera_posts-posts.begin()+1<<std::endl;
        double overrun_distance =  point2PointDistance(*(itera_posts+2),now_pose.pose );
        double overrun_angle = point2PointYaw(*(itera_posts+2),now_pose.pose);
        if(overrun_distance < ZERO && overrun_angle < ZERO*5)
        {
          std::cout<<"越过了T+1到达T+2"<<std::endl;
          itera_posts +=2;
          if(itera_posts+2<posts.end())
            gotoLocalTPlusI(2);
          else if(itera_posts+2 == posts.end())
            gotoLocalTPlusI(1);
        }
      }
    }
        
        
        
  }

    
    /*
    //偏离路线
    if(globlebegin)
    {
      static geometry_msgs::Pose arrive_point;
      static geometry_msgs::Pose goal_point = *itera_posts;
      
      if(point2PointDistance(posestamped->pose,goal_point)<ZERO)//到达目标点附近
      {
          path_correction =false;
      }


      arrive_point = *itera_posts;
      if(itera_posts+2<posts.end())
      {
        goal_point = *(itera_posts+2);
      }
      else if(itera_posts+2==posts.end())
      {
        goal_point = *(itera_posts+1);
      }
      else
      {
        std::cout<<"error 偏离路线部分出意料外错误";
      }
      
      double std_distance = point2LineDistance(posestamped->pose,arrive_point,goal_point);
      if(std_distance>ZERO)//回到原位 矫正标记为true
      {
        std::cout<<"路线偏离"<<std_distance<<"米返回到达点"<<std::endl;
        getMsgFromlist(arrive_point,action_goal);
        chatter_pub->publish(action_goal);  
        path_correction = true;      
      }
      
      if(path_correction)//原地待机在次去目的地
      {
        ros::Rate rate(0.25);
        rate.sleep();
        getMsgFromlist(goal_point,action_goal);
        chatter_pub->publish(action_goal);   
      }
    }
    */
}
void getMsgFromlist(geometry_msgs::Pose point,geometry_msgs::PoseStamped& action_goal)
{
  count++;
  action_goal.header.seq=count;
  action_goal.header.frame_id="map";
  action_goal.header.stamp = ros::Time::now();
  action_goal.pose = point;
}
void getMsgFromlist(std::vector<geometry_msgs::Pose>::iterator point,geometry_msgs::PoseStamped& action_goal)
{
  count++;
  action_goal.header.seq=count;
  action_goal.header.frame_id="map";
  action_goal.header.stamp = ros::Time::now();
  action_goal.pose = *point;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "along2");//指定节点名称
  
  readPosts(globle_posts,"posts.txt");
  itera_posts=posts.begin();
  itera_globle_posts = globle_posts.begin();
  //move_base_msgs::MoveBaseActionGoal action_goal;//发送信息

  ros::NodeHandle n; //节点句柄发送句柄
  if(n.getParam("FX",FX))
  {
    if(FX == 0)
      std::cout<<"autolabor2.5启动成功"<<std::endl;
    else if(FX == 1)
      std::cout<<"autolaborPro启动成功"<<std::endl;
    else
      std::cout<<"FX参数错误"<<std::endl;

  }
  else
    std::cout<<"读取参数错误"<<std::endl;
  



  ros::Publisher chatterp =n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  chatter_pub = &chatterp;

  ros::Subscriber sub_result = n.subscribe("/move_base/result", 10, chatterCallback);
  ros::Subscriber sub_pose = n.subscribe("/robot_pose",10, postCallback);
  ros::Subscriber sub_scan=n.subscribe("/scan",10,scanCallback);

  ros::Rate loop_rate(0.5);//ros::Rate对象指定自循环频
  loop_rate.sleep();  //第一次发布前给master通知的时间

 
  getMsgFromlist(itera_globle_posts,action_goal);//获得信息 
   
  chatter_pub->publish(action_goal);
  std::cout<<"初始节点发送成功"<<std::endl;


  ros::spin();
  int n_count=0;
  ros::Rate rate(5);
  // while(ros::ok())
  // {
  //   ros::spinOnce();
  //   rate.sleep();
  //   n_count++;
  //   // if(!find_obstacle && n_count%15==0)
  //   //   chatter_pub->publish(action_goal);
  // }

  return 0;
}