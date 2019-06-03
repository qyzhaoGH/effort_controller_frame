#ifndef MY_CONTROLLER__JOINT_CONTROLLER_H
#define MY_CONTROLLER__JOINT_CONTROLLER_H
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
//#include <control_msgs/JointControllerState.h>
#include <my_controller/JointControllerState.h>
#include <my_controller/JointError.h>

namespace my_controller
{

class JointController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

struct Commands
  {
   double position_;
   double velocity_; 
   double effort_;
  };

  JointController();
  ~JointController();
 bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

 void setCommand(double eff_command);
  
 double computeCommand(double error, ros::Duration dt);

 double computeCommand(double error, double error_dot_, ros::Duration dt);

 void starting(const ros::Time& time);

 void update(const ros::Time& time, const ros::Duration& period);
 
 std::string getJointName();

 double getPosition();
 
 void setCommandTransit(const geometry_msgs::Vector3ConstPtr& msg);

 void enforceJointLimits(double &command);

  hardware_interface::JointHandle joint_;
  urdf::JointConstSharedPtr joint_urdf_;
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;

private:
  int loop_count_;
  //double p_error_last_; /**< _Save position state for derivative state calculation. */
  //double p_error_; /**< Position error. */
  //double i_error_; /**< Integral of position error. */
  //double d_error_; /**< Derivative of position error. */
  double cmd_;     /**< Command to send. */
  bool cmd_diff; 
  int count_;
  int num1;
  int num2;
  double p;
  double v;
  double dt;
  double step;
  //std::stringstream ss;
  std_msgs::Float64 cmd; 
  std_msgs::Float64 num1_;
  std_msgs::Float64 num2_;
  /**boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      my_controller::JointControllerState> > controller_state_publisher_ ;
  
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      my_controller::JointError> > error_publisher_ ;  **/
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      std_msgs::Float64> > position_publisher_ ;
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      std_msgs::Float64> > velocity_publisher_ ;
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      geometry_msgs::Vector3> > state_pub ;

  ros::Subscriber sub_command_;
  ros::Publisher cmd_publisher_ ;
  ros::Publisher num1_pub_;
  ros::Publisher num2_pub_;

};

}

#endif
