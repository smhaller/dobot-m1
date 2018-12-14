#include "dobot_m1_hw_interface.h"

using namespace dobot_m1_hw_interface;


// Used to convert seconds elapsed to nanoseconds
static const double BILLION = 1000000000.0;


int main(int argc, char**argv)
{
  std::string name = "dobot_m1";
  ros::init(argc, argv, name);
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create the hardware interface
  boost::shared_ptr<DobotM1HWInterface> dobot_m1_hw_interface
    (new DobotM1HWInterface(nh));
  dobot_m1_hw_interface->init();

  // Create controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager;
  controller_manager.reset(new controller_manager::ControllerManager(dobot_m1_hw_interface.get(), nh));

  // Timing
  double cycle_time_error_threshold_=0.05;
  ros::Duration elapsed_time;
  // Note: I did not find a possibility to Ctrl the Motors directly (with the provided library) - so this 
  // is no 'real' Controller because of the PTPCmd of Dobot - 20Hz is more or less the upper limit 
  double loop_hz=20.0;
  struct timespec last_time;
  struct timespec current_time;
  ros::NodeHandle rpsnh(nh, "/hardware_interface");

  ROS_INFO_STREAM("Loop hz: " << std::to_string(loop_hz));

  // Get current time for use with first update
  clock_gettime(CLOCK_MONOTONIC, &last_time);

  ros::Duration desired_update_period_ = ros::Duration(1 / loop_hz);

  // Start control loop
  ros::Rate rate(loop_hz);
  while(ros::ok())
  {
    // Get change in time
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time =  ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    // Error check cycle time
    const double cycle_time_error = (elapsed_time - desired_update_period_).toSec();
    if (cycle_time_error > cycle_time_error_threshold_)
    {
      ROS_WARN_STREAM_NAMED(name, "Cycle time exceeded error threshold by: " << cycle_time_error << ", cycle time: " << elapsed_time << ", threshold: " << cycle_time_error_threshold_);
    }

    // Read
    dobot_m1_hw_interface->read(elapsed_time);

    // Control
    controller_manager->update(ros::Time::now(), elapsed_time);

    // Write 
    dobot_m1_hw_interface->write(elapsed_time);

    rate.sleep();
  } // end while
  
  dobot_m1_hw_interface->disconnect();


} // EOF
