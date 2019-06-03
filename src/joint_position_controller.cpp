#include <joint_position_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace my_controller   {

JointPositionController::JointPositionController()
  : loop_count_(0)
{}

JointPositionController::~JointPositionController()
{
  sub_command_.shutdown();
}

bool JointPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  /**Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;  */

  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<my_controller::JointControllerState>(n, "state", 1));
  
  error_publisher_.reset(
    new realtime_tools::RealtimePublisher<my_controller::JointError>(n, "error", 1));
     //new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(n, "state", 1));
  // Start command subscriber

  pos_publisher_=n.advertise<std_msgs::Float64>("pos",1);
  cmd_publisher_=n.advertise<std_msgs::Float64>("cmd",1);
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointPositionController::setCommandTransit, this);

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
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


std::string JointPositionController::getJointName()
{
  return joint_.getName();
}

double JointPositionController::getPosition()
{
  return joint_.getPosition();
}

void JointPositionController::setCommand(double pos_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);
}
// Set the joint position command
/**void JointController::setCommand(double eff_command)
{
  command_struct_.effort_ = eff_command;
 
  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);
}  **/



void JointPositionController::starting(const ros::Time& time)
{
  
  // double eff_command = joint_.getEffort();
  double pos_command = joint_.getPosition();
  enforceJointLimits(pos_command);

  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false;

  // command_struct_.effort_ = eff_command;

  command_.initRT(command_struct_);
  loop_count_=0;
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  cmd_ = 0.0;
  commanded_effort=0.0;
}

void JointPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  //command_struct_ = *(command_.readFromRT());
  //double command_effort = command_struct_.effort_;
  
  // Make sure joint is within limits if applicable
  //enforceJointLimits(command_position);
  
  command_struct_ = *(command_.readFromRT());
  double command_position = command_struct_.position_;
  double command_velocity = command_struct_.velocity_;
  bool has_velocity_ =  command_struct_.has_velocity_;

  
  double error, vel_error;
  
 
  double current_position = joint_.getPosition();

  enforceJointLimits(command_position);
  
  error = angles::shortest_angular_distance(current_position, command_position);

  if (has_velocity_)
  {
    // Compute velocity error if a non-zero velocity command was given
    vel_error = command_velocity - joint_.getVelocity();

    // Set the PID error and compute the PID command with nonuniform
    // time step size. This also allows the user to pass in a precomputed derivative error.
    commanded_effort = computeCommand(error, vel_error, period);
  }  
  else
  {
    // Set the PID error and compute the PID command with nonuniform
    // time step size.
    commanded_effort = computeCommand(error, period);
  }                                                                
  

  joint_.setCommand(commanded_effort);

  pos.data=joint_.getPosition();
  cmd.data=commanded_effort;
  pos_publisher_.publish(pos);
  cmd_publisher_.publish(cmd);

  /** publish state
  if (loop_count_ % 10 == 0)
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
  }
  loop_count_++; **/


}

void JointPositionController::setCommandTransit(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
void JointPositionController::enforceJointLimits(double &command)
{
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
  {
    if( command > joint_urdf_->limits->upper ) // above upper limnit
    {
      command = joint_urdf_->limits->upper;
    }
    else if( command < joint_urdf_->limits->lower ) // below lower limit
    {
      command = joint_urdf_->limits->lower;
    }
  }
}

double JointPositionController::computeCommand(double error, ros::Duration dt)
{

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
    return 0.0;

  double error_dot = d_error_;

  // Calculate the derivative error
  if (dt.toSec() > 0.0)
  {
    error_dot = (error - p_error_last_) / dt.toSec();
    p_error_last_ = error;
  }

  return computeCommand(error, error_dot, dt);
}

double JointPositionController::computeCommand(double error, double error_dot, ros::Duration dt)
{
  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
    return 0.0;
  
  p_error_=error;
  // Calculate proportional contribution to command
  double p_term = 100.0 * p_error_;

  // Calculate the integral of the position error
  i_error_ += dt.toSec() * p_error_;

  // Calculate integral contribution to command
  double i_term = 0.01 * i_error_;
  
  d_error_ = error_dot;
  // Calculate derivative contribution to command
  double d_term = 10.0 * d_error_;

  // Compute the command
  cmd_ = p_term + i_term + d_term;
  return cmd_;
}
} // namespace



PLUGINLIB_EXPORT_CLASS( my_controller::JointPositionController, controller_interface::ControllerBase)
