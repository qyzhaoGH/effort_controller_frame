#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");

  ros::NodeHandle n;

  ros::Publisher pos_pub = n.advertise<std_msgs::Float64>("/f5manipulator/joint2_position_conoller/position", 10);
  ros::Publisher vel_pub = n.advertise<std_msgs::Float64>("/f5manipulator/joint2_position_conoller/velocity", 10);

  ros::Subscriber com_sub = n.subscribe<std_msgs::Float64>("/f5manipulator/joint2_position_conoller/command", 10, chatterCallback);

  ros::Rate loop_rate(10);
  int flag = 0;
  int pos=1;
  int vel=0;
  int com=0;
  std_msgs::Float64 pos_;
  std_msgs::Float64 vel_;
  
  pos_.data=pos;
  pos_pub.publish(pos_);

  while (ros::ok())
  {
  
  if (flag)
  {
  pos++;
  //vel++;
  
  pos_.data=pos;
  vel_.data=com;
     
  pos_pub.publish(pos_);
  vel_pub.publish(vel_);
  flag=0;
  }

  }

  return 0;

}


void chatterCallback(const std_msgs::Float64ConstPtr& msg)
{
flag=1;
com++;
}
