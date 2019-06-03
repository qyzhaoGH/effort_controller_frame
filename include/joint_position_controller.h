
#ifndef MY_CONTROLLER__JOINT_CONTROLLER_H
#define MY_CONTROLLER__JOINT_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
  /*#include <control_toolbox/pid.h>*/
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

#include <std_msgs/Float64.h>
//#include <control_msgs/JointControllerState.h>
#include <my_controller/JointControllerState.h>
#include <my_controller/JointError.h>
 
namespace my_controller
{

class JointPositionController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

  /**
   * \brief Store position and velocity command in struct to allow easier realtime buffer usage
   */
  struct Commands
  {
   double position_;
   double velocity_; 
   double effort_;
   bool has_velocity_;
  };

  JointPositionController();
  ~JointPositionController();

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */  
  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param command
   */
  void setCommand(double pos_command);
  
  double computeCommand(double error, ros::Duration dt);

  double computeCommand(double error, double error_dot_, ros::Duration dt);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *        Also supports a target velocity
   *
   * \param pos_target - position setpoint
   * \param vel_target - velocity setpoint
   */
  void setCommand(double pos_target, double vel_target);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);
  
  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Get the PID parameters
   */
  /**void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  /**
   * \brief Get the PID parameters
   */
  /**void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

  /**
   * \brief Print debug info to console
   */
  /**void printDebug();

  /**
   * \brief Get the PID parameters
   */
  /**void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);

  /**
   * \brief Get the name of the joint this controller uses
   */
  std::string getJointName();

  /**
   * \brief Get the current position of the joint
   * \return current position
   */
  double getPosition();

  
  hardware_interface::JointHandle joint_;
  urdf::JointConstSharedPtr joint_urdf_;
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer
  
  
private:
  int loop_count_;
  double p_error_last_; /**< _Save position state for derivative state calculation. */
  double p_error_; /**< Position error. */
  double i_error_; /**< Integral of position error. */
  double d_error_; /**< Derivative of position error. */
  double cmd_;     /**< Command to send. */
  double commanded_effort;
  std_msgs::Float64 pos;
  std_msgs::Float64 cmd;
  

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      my_controller::JointControllerState> > controller_state_publisher_ ;
  
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      my_controller::JointError> > error_publisher_ ;
  ros::Publisher pos_publisher_;
  ros::Publisher cmd_publisher_;
  ros::Subscriber sub_command_;

  /**
   * \brief Callback from /command subscriber for setpoint
   */
  void setCommandTransit(const std_msgs::Float64ConstPtr& msg);

  /**
   * \brief Check that the command is within the hard limits of the joint. Checks for joint
   *        type first. Sets command to limit if out of bounds.
   * \param command - the input to test
   */
  void enforceJointLimits(double &command);

};

} // namespace

#endif
