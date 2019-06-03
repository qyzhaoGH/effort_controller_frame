
#include <joint_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace my_controller   {

JointController::JointController()
  : loop_count_(0)
{}

JointController::~JointController()
{
  sub_command_.shutdown();
}

bool JointController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

// 修改区
  /**controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<my_controller::JointControllerState>(n, "state", 1));
  
  error_publisher_.reset(
    new realtime_tools::RealtimePublisher<my_controller::JointError>(n, "error", 1));  **/
  
  position_publisher_.reset(
    new realtime_tools::RealtimePublisher<std_msgs::Float64>(n, "position", 5));
  velocity_publisher_.reset(
    new realtime_tools::RealtimePublisher<std_msgs::Float64>(n, "velocity", 5));

  state_pub.reset(
    new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(n, "newstate", 5));
  
  num1_pub_=n.advertise<std_msgs::Float64>("num1", 5);
  num2_pub_=n.advertise<std_msgs::Float64>("num2", 5);
  
  cmd_publisher_ = n.advertise<std_msgs::Float64>("cmd", 5);
  sub_command_ = n.subscribe<geometry_msgs::Vector3>("command", 5, &JointController::setCommandTransit, this);
//修改区

  joint_ = robot->getHandle(joint_name);
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", n))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}


void JointController::starting(const ros::Time& time)
{
  double eff_command = joint_.getEffort();
  //double pos_command = joint_.getPosition();
  //enforceJointLimits(pos_command);
  
  //command_struct_.position_ = pos_command;
  //command_struct_.has_velocity_ = false;
  command_struct_.effort_ = eff_command;
  num1=0.0;
  num2=0.0;
  count_=1;
  p=0.0;
  v=0.0;
  dt=1.0;
  step=1.0;

  command_.initRT(command_struct_);
  loop_count_=1;

  cmd_diff=false;
  /**p_error_last_ = 0.0;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  cmd_ = 0.0; **/

}

void JointController::update(const ros::Time& time, const ros::Duration& period)
{
  p=joint_.getPosition();
  v=joint_.getVelocity();
  //dt=period.toSec();
  state_pub->trylock();
  state_pub->msg_.x=p;
  state_pub->msg_.y=v;
  state_pub->msg_.z=step;
  state_pub->unlockAndPublish();
  loop_count_=1;
  /**position_publisher_->trylock();
  position_publisher_->msg_.data=joint_.getPosition();
  position_publisher_->unlockAndPublish();
  velocity_publisher_->trylock();
  velocity_publisher_->msg_.data=joint_.getVelocity();
  velocity_publisher_->unlockAndPublish();**/
  // Make sure joint is within limits if applicable
  //enforceJointLimits(command_position);
  
  //ss << count_;
  //ROS_INFO("%s", ss.str().c_str());
  //count.data=count_;
  //count_publisher_.publish(count);
  int t=1;
  while (count_)
  {
      t=1; 
      state_pub->trylock();
      state_pub->unlockAndPublish();
       
  }
  
  //loop_count_=1;
  
  //loop_count_=0;
  command_struct_ = *(command_.readFromRT());
  double command_effort = command_struct_.effort_;
  double commanded_effort=command_effort;
  
  /**if (abs(commanded_effort)>50.0)
  commanded_effort=50.0;
  
  if (abs(commanded_effort)<(0.0-50.0))
  commanded_effort=(0.0-50.0);  **/

  joint_.setCommand(commanded_effort);
  count_=1;

  cmd.data=commanded_effort;
  cmd_publisher_.publish(cmd);
  step=step+1;
  
  // publish state
  /**if (loop_count_ % 100 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      //controller_state_publisher_->msg_.set_point = command_position;
      controller_state_publisher_->msg_.process_value = joint_.getPosition();
      controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = commanded_effort;
      
      controller_state_publisher_->unlockAndPublish();
    }

    if(error_publisher_ && error_publisher_->trylock())
    {
      error_publisher_->msg_.p_error_ = p_error_;
      error_publisher_->msg_.i_error_ = i_error_;
      error_publisher_->msg_.d_error_ = d_error_;
      error_publisher_->msg_.p_error_last_ = p_error_last_;
      error_publisher_->msg_.location_=joint_.getPosition();
      error_publisher_->msg_.effort_=joint_.getEffort();
      error_publisher_->unlockAndPublish();
    }  
    /**if(position_publisher_ && position_publisher_->trylock())
    {
      position_publisher_->msg_.data=joint_.getPosition();
      position_publisher_->unlockAndPublish();
    }
    if(velocity_publisher_ && velocity_publisher_->trylock())
    {
      velocity_publisher_->msg_.data=joint_.getVelocity();
      velocity_publisher_->unlockAndPublish();
    }  **/
 /** }
  loop_count_++;**/
  num1_.data=num1;
  num1_pub_.publish(num1_);
  num1++;
  dt=period.toSec();
  
}

std::string JointController::getJointName()
{
  return joint_.getName();
}

double JointController::getPosition()
{
  return joint_.getPosition();
}

/**void JointController::setCommand(double eff_command)
{
  command_struct_.effort_ = eff_command;
  
  command_.writeFromNonRT(command_struct_);
  if (loop_count_)
  count_=0;
  
  num2_.data=num2;
  num2_pub_.publish(num2_);
  num2++;
  
  
}
void JointController::setCommandTransit(const geometry_msgs::Vector3ConstPtr& msg)
{
  if((p==msg->x) && (v==msg->y))
  cmd_diff=true;
  
  
  if(cmd_diff)
  {
  setCommand(msg->z);
  cmd_diff=false;
  }

  
 
  
}
**/
void JointController::setCommand(double eff_command)
{
  command_struct_.effort_ = eff_command;
  
  command_.writeFromNonRT(command_struct_);
  count_=0;  
}

void JointController::setCommandTransit(const geometry_msgs::Vector3ConstPtr& msg)
{
  if((loop_count_==1)&&(msg->y==step))
  {cmd_diff=true;
  loop_count_=0;}
  
  
  if(cmd_diff)
  {
  setCommand(msg->x);
  cmd_diff=false;
  num2_.data=num2;
  num2_pub_.publish(num2_);
  num2++;
  } 
  
}



}
PLUGINLIB_EXPORT_CLASS( my_controller::JointController, controller_interface::ControllerBase)
