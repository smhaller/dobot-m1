// ROS
#include <limits>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include <iostream>       // std::cout, std::endl
# define M_PI           3.14159265358979323846  /* pi */

// Dobot M1 ros control hardware interface
#include "dobot_m1_hw_interface.h"

// Dobot M1
#include "DobotDll.h"


namespace dobot_m1_hw_interface
{
    DobotM1HWInterface::DobotM1HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) : 
    name_("dobot_m1_hw_interface"),
    nh_(nh),
    use_rosparam_joint_limits_(false),
    use_soft_limits_if_available_(false)
   {
    // Check if the URDF model needs to be loaded
    if (urdf_model == NULL)
      loadURDF(nh, "/robot_description");
    else
      urdf_model_ = urdf_model;

    // Load rosparams
    ros::NodeHandle rpnh(nh_, "hardware_interface");
    std::size_t error = 0;
    error += !rosparam_shortcuts::get("hardware_interface", rpnh, "joints", joint_names_);
    rosparam_shortcuts::shutdownIfError("hardware_interface", error);

    // Dobotr M1 Parameters...
    const std::string packagePath  = ros::package::getPath("dobot_m1_hw");
    std::string deviceName;
    std::string deviceIP;

    ros::param::param<std::string>("/dobby/deviceName",deviceName,"Dobby");
    ros::param::param<std::string>("/dobby/deviceIP",deviceIP,"192.168.1.192");

    uint64_t queuedCmdIndex;

    // Connect to Dobot 
    if (ConnectDobot(deviceIP.c_str(), 115200, 0, 0) == DobotCommunicate_NoError) {
        ROS_INFO("Opened Dobot Communication");
    } else {
        ROS_ERROR("Invalid port name or Dobot is occupied by other application or something else :)");
    }
    // Set Commuinication timeout in ms 
    uint32_t timeout=20;
    SetCmdTimeout(timeout);

    // Set some Parameter and Initialisation
    // First clear the command queue
    if (SetQueuedCmdClear() == DobotCommunicate_NoError) {
           ROS_INFO("CMD Queue Cleared");
    }
    // and start the exec queue
    if (SetQueuedCmdStartExec() == DobotCommunicate_NoError) {
           ROS_INFO("CMD Queue Started");
    }

    // Set Coord/Jump/Accl/Vel/Joint Parameters
    PTPCoordinateParams coordparams;
    coordparams.xyzVelocity = 100; 
    coordparams.rVelocity = 100; 
    coordparams.xyzAcceleration = 100; 
    coordparams.rAcceleration = 100;
    if (SetPTPCoordinateParams(&coordparams, true, &queuedCmdIndex) == DobotCommunicate_NoError) {
      ROS_INFO("Dobot Coordinate Params Set.");
    }

    PTPJumpParams jumpparams;
    jumpparams.jumpHeight = 20;
    jumpparams.zLimit = 200;
    if (SetPTPJumpParams(&jumpparams, true, &queuedCmdIndex) == DobotCommunicate_NoError) {
      ROS_INFO("Dobot Junp Params Set.");
    }

    // Set Acc and Speed
    PTPCommonParams commonparams;
    commonparams.velocityRatio=100;
    commonparams.accelerationRatio=100;
    if (SetPTPCommonParams(&commonparams, true, &queuedCmdIndex) == DobotCommunicate_NoError) {
      ROS_INFO("Dobot Common Params Set.");
    }

    PTPJointParams jointparams;
    for (int i = 0; i < 4; i++) {
        jointparams.velocity[i] = 100;
        jointparams.acceleration[i] = 100;
    }

    if (SetPTPJointParams(&jointparams, true, &queuedCmdIndex) == DobotCommunicate_NoError) {
      ROS_INFO("Dobot Joint Params Set.");

    }

    // For fun - set Dobot M1 Name
    if (SetDeviceName(deviceName.c_str()) == DobotCommunicate_NoError) {
      ROS_INFO("Dobot is Running.");
    }
  }

  void DobotM1HWInterface::init()
  {
     
    num_joints_ = joint_names_.size();

    // Status
    joint_position_.resize(num_joints_, 0.0);
    joint_velocity_.resize(num_joints_, 0.0);
    joint_effort_.resize(num_joints_, 0.0);

    // Command
    joint_position_command_.resize(num_joints_, 0.0);
    joint_velocity_command_.resize(num_joints_, 0.0);
    joint_effort_command_.resize(num_joints_, 0.0);

    // Limits
    joint_position_lower_limits_.resize(num_joints_, 0.0);
    joint_position_upper_limits_.resize(num_joints_, 0.0);
    joint_velocity_limits_.resize(num_joints_, 0.0);
    joint_effort_limits_.resize(num_joints_, 0.0);

    // Initialize interfaces for each joint
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);

      // Create joint state interface
      joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[joint_id], &joint_position_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));

      // Add command interfaces to joints
      // TODO: decide based on transmissions?
      hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);
      position_joint_interface_.registerHandle(joint_handle_position);

      hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]);
      velocity_joint_interface_.registerHandle(joint_handle_velocity);

      hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);
      effort_joint_interface_.registerHandle(joint_handle_effort);

      // Load the joint limits
      registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
    }  // end for each joint

    registerInterface(&joint_state_interface_);     // From RobotHW base class.
    registerInterface(&position_joint_interface_);  // From RobotHW base class.
    registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
    registerInterface(&effort_joint_interface_);    // From RobotHW base class.

    ROS_INFO_STREAM_NAMED(name_, "Dobot M1 Hardware Interface Ready.");
  }

  void DobotM1HWInterface::read(ros::Duration &elapsed_time)
  {
    Pose pose;
    if ( GetPose(&pose) == DobotCommunicate_NoError) {
      joint_position_.clear();
      // Not Cartesian Pose
      //joint_position_.push_back(pose.x);
      //joint_position_.push_back(pose.y);
      //joint_position_.push_back(pose.z);
      //joint_position_.push_back(pose.r);
      /* But push back joint angles - J3 [2] is Z-Axis !! */
      joint_position_.push_back(pose.jointAngle[2]/1000);
      joint_position_.push_back(pose.jointAngle[0]*M_PI/180);
      joint_position_.push_back(pose.jointAngle[1]*M_PI/180);
      joint_position_.push_back(pose.jointAngle[3]*M_PI/180);
      //std::cout << "Got Position: " << joint_position_command_.at(0) << " " << joint_position_command_.at(1) << " " << joint_position_command_.at(2) << " " << joint_position_command_.at(3) << std::endl;
      //ROS_DEBUG_STREAM_NAMED(name_, joint_positions_string);
    }
  }

  void DobotM1HWInterface::write(ros::Duration &elapsed_time)
  {
    // Safety
    enforceLimits(elapsed_time);
    // Clear not executed requests and set new target position
    PTPCmd cmd;
    uint64_t queuedCmdIndex;
    uint64_t execCmdIndex=0;
    /* PTP Modes (MOVJ: Joint movement. From point A to point B, each joint will run from initial angle
     * to its target angle, MOVL: Rectilinear movement. The joints will perform a straight line trajectory from
     * point A to point B
     * JUMP_XYZ, //JUMP mode, (x,y,z,r) is the target point in Cartesian
     * MOVJ_XYZ, //MOVJ mode, (x,y,z,r) is the target point in Cartesian
     * MOVL_XYZ, //MOVL mode, (x,y,z,r) is the target point in Cartesian
     * JUMP_ANGLE, //JUMP mode, (x,y,z,r) is the target point in Joint
     * MOVJ_ANGLE, //MOVJ mode, (x,y,z,r) is the target point in Joint
     * MOVL_ANGLE, //MOVL mode, (x,y,z,r) is the target point in Joint
     * MOVJ_INC, //MOVJ mode, (x,y,z,r) is the angle increment in Joint
     * MOVL_INC, //MOVL mode, (x,y,z,r) is the Cartesian coordinate increment in Joint 
     * MOVJ_XYZ_INC, //MOVJ mode, (x,y,z,r) is the Cartesian coordinate increment in Cartesian 
     * JUMP_MOVL_XYZ, //JUMP mode, (x,y,z,r) is the Cartesian coordinate increment in Cartesian
       */
    cmd.ptpMode = 4;
    cmd.x = joint_position_command_.at(1)*180/M_PI;
    cmd.y = joint_position_command_.at(2)*180/M_PI;
    cmd.z = joint_position_command_.at(0)*1000;
    cmd.r = joint_position_command_.at(3)*180/M_PI;
       //Send the position command
    if ( SetPTPCmd(&cmd, true, &queuedCmdIndex) != DobotCommunicate_NoError) {
	    ROS_INFO_STREAM_NAMED(name_, "Sending Position: Some Error Occured.");
    }
    GetQueuedCmdCurrentIndex(&execCmdIndex);
    // If queue is behind - cleanup
    if (execCmdIndex+2 < queuedCmdIndex){
        std::cout << "Exec: " <<  execCmdIndex << " Send: " << queuedCmdIndex << std::endl;
        // let the command finish: if you just clear the cmd stack the executed cmd gets stopped.
        SetQueuedCmdStopExec();
        SetQueuedCmdClear();
        SetQueuedCmdStartExec();
        // resend the current PTP Cmd - and don't care any more
        SetPTPCmd(&cmd, true, &queuedCmdIndex);
    }
  }

  void DobotM1HWInterface::enforceLimits(ros::Duration &period)
  {
    // Saturation limits
    pos_jnt_sat_interface_.enforceLimits(period);
    // vel_jnt_sat_interface_.enforceLimits(period);
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits
    pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
  }

  void DobotM1HWInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position, const hardware_interface::JointHandle &joint_handle_velocity, const hardware_interface::JointHandle &joint_handle_effort, std::size_t joint_id)
  {
    // Default values
    joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
    joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();
    joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();
    joint_effort_limits_[joint_id] = std::numeric_limits<double>::max();

    // Limits datastructures
    joint_limits_interface::JointLimits joint_limits;     // Position
    joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
    bool has_joint_limits = false;
    bool has_soft_limits = false;

    // Get limits from URDF
    if (urdf_model_ == NULL)
    {
      ROS_WARN_STREAM_NAMED(name_, "No URDF model loaded, unable to get joint limits");
      return;
    }

    // Get limits from URDF
    urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

    // Get main joint limits
    if (urdf_joint == NULL)
    {
      ROS_ERROR_STREAM_NAMED(name_, "URDF joint not found " << joint_names_[joint_id]);
      return;
    }

    // Get limits from URDF
    if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
    {
      has_joint_limits = true;
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF position limits [" << joint_limits.min_position << ", " << joint_limits.max_position << "]");
      if (joint_limits.has_velocity_limits)
        ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF velocity limit [" << joint_limits.max_velocity << "]");
    }
    else
    {
      if (urdf_joint->type != urdf::Joint::CONTINUOUS)
        ROS_WARN_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have a URDF position limit");
    }

    // Get limits from ROS param
    if (use_rosparam_joint_limits_)
    {
      if (joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, joint_limits))
      {
        has_joint_limits = true;
        ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has rosparam position limits [" << joint_limits.min_position << ", " << joint_limits.max_position << "]");
        if (joint_limits.has_velocity_limits)
          ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has rosparam velocity limit [" << joint_limits.max_velocity << "]");
      }  // the else debug message provided internally by joint_limits_interface
    }

    // Get soft limits from URDF
    if (use_soft_limits_if_available_)
    {
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
      {
        has_soft_limits = true;
        ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has soft joint limits.");
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have soft joint limits");
      }
    }

    // Quit we we haven't found any limits in URDF or rosparam server
    if (!has_joint_limits)
    {
      return;
    }

    // Copy position limits if available
    if (joint_limits.has_position_limits)
    {
      // Slighly reduce the joint limits to prevent floating point errors
      joint_limits.min_position += std::numeric_limits<double>::epsilon();
      joint_limits.max_position -= std::numeric_limits<double>::epsilon();

      joint_position_lower_limits_[joint_id] = joint_limits.min_position;
      joint_position_upper_limits_[joint_id] = joint_limits.max_position;
    }

    // Copy velocity limits if available
    if (joint_limits.has_velocity_limits)
    {
      joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
    }

    // Copy effort limits if available
    if (joint_limits.has_effort_limits)
    {
      joint_effort_limits_[joint_id] = joint_limits.max_effort;
    }

    if (has_soft_limits)  // Use soft limits
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Using soft saturation limits");
      const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position, joint_limits, soft_limits);
      pos_jnt_soft_limits_.registerHandle(soft_handle_position);
      const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity, joint_limits, soft_limits);
      vel_jnt_soft_limits_.registerHandle(soft_handle_velocity);
      const joint_limits_interface::EffortJointSoftLimitsHandle soft_handle_effort(joint_handle_effort, joint_limits, soft_limits);
      eff_jnt_soft_limits_.registerHandle(soft_handle_effort);
    }
    else  // Use saturation limits
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Using saturation limits (not soft limits)");

      const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
      pos_jnt_sat_interface_.registerHandle(sat_handle_position);

      const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);
      vel_jnt_sat_interface_.registerHandle(sat_handle_velocity);

      const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, joint_limits);
      eff_jnt_sat_interface_.registerHandle(sat_handle_effort);
    }
  }

  void DobotM1HWInterface::reset()
  {
    // Reset joint limits state, in case of mode switch or e-stop
    pos_jnt_sat_interface_.reset();
    pos_jnt_soft_limits_.reset();
  }

  void DobotM1HWInterface::printState()
  {
    // WARNING: THIS IS NOT REALTIME SAFE
    // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
    ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
  }

  std::string DobotM1HWInterface::printStateHelper()
  {
    std::stringstream ss;
    std::cout.precision(15);

    for (std::size_t i = 0; i < num_joints_; ++i)
    {
      ss << "j" << i << ": " << std::fixed << joint_position_[i] << "\t ";
      ss << std::fixed << joint_velocity_[i] << "\t ";
      ss << std::fixed << joint_effort_[i] << std::endl;
    }
    return ss.str();
  }

  std::string DobotM1HWInterface::printCommandHelper()
  {
    std::stringstream ss;
    std::cout.precision(15);
    ss << "    position     velocity         effort  \n";
    for (std::size_t i = 0; i < num_joints_; ++i)
    {
      ss << "j" << i << ": " << std::fixed << joint_position_command_[i] << "\t ";
      ss << std::fixed << joint_velocity_command_[i] << "\t ";
      ss << std::fixed << joint_effort_command_[i] << std::endl;
    }
    return ss.str();
  }

  void DobotM1HWInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
  {
    std::string urdf_string;
    urdf_model_ = new urdf::Model();

    // search and wait for robot_description on param server
    while (urdf_string.empty() && ros::ok())
    {
      std::string search_param_name;
      if (nh.searchParam(param_name, search_param_name))
      {
        ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << search_param_name);
        nh.getParam(search_param_name, urdf_string);
      }
      else
      {
        ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << param_name);
        nh.getParam(param_name, urdf_string);
      }
      usleep(100000);
    }

    if (!urdf_model_->initString(urdf_string))
      ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
    else
      ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
  }
  void DobotM1HWInterface::disconnect()
  {
  DisconnectDobot();
  }
}
