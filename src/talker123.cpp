#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/Num.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "zsy_talker");//指定节点名称


  ros::NodeHandle n;//节点句柄

  ros::Publisher chatter_pub = n.advertise<beginner_tutorials::Num>("chatter", 1000);//准备发布话题 话题类型模板给出,话题名,话题长度参数给出

  ros::Rate loop_rate(1);//ros::Rate对象指定自循环频率

  //spin时最先执行
  //ros::Subscriber sub = n.subscribe("chatter",1000,chatterCallback);
  
  int count = 0;
  /*
1 SIGINT 被触发 (Ctrl-C)
2 被另一同名节点踢出 ROS 网络
3 ros::shutdown() 被程序的另一部分调用
4 节点中的所有 ros::NodeHandles 都已经被销毁 
时ros::ok()为false
*/
  
  

 // while(ros::ok()){ //只有ok循环spin才发!!!!!
    beginner_tutorials::Num num;
    num.Num=0;
    loop_rate.sleep();
    chatter_pub.publish(num);
    std::cout<<"123";
    ROS_INFO("%s","发送成功");
    /* code */
    //loop_rate.sleep();
    // ros::spinOnce();

    // if (count>0) {
    //   break;
    //   /* code */
    // }
    
    count++;
  //}
  
  // while (ros::ok())
  // {
  //   /**
  //    * This is a message object. You stuff it with data, and then publish it.
  //    */
  //   std_msgs::String msg;

  //   std::stringstream ss;
  //   ss << "hello world " << count;
  //   msg.data = ss.str();

  //   ROS_INFO("%s", msg.data.c_str());//代替printf/cout

  //   /**
  //    * The publish() function is how you send messages. The parameter
  //    * is the message object. The type of this object must agree with the type
  //    * given as a template parameter to the advertise<>() call, as was done
  //    * in the constructor above.
  //    */
  //   chatter_pub.publish(msg);
  //   chatter_pub.publish(msg);

  //   ros::spinOnce(); //执行回调函数

  //   loop_rate.sleep();
  //   ++count;
  // }


  return 0;
}