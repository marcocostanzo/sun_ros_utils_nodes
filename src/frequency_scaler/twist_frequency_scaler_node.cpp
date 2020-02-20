#include "sun_ros_utils_nodes/FrequencyScaler.h"
#include "geometry_msgs/TwistStamped.h"

int main(int argc, char *argv[]) {
  
  // ROS INIT - Use a coustom signinthandler
  ros::init(argc, argv, "wrench_frequency_scaler");

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  sun::FrequencyScalerNode<geometry_msgs::TwistStamped> frequency_scaler(nh_public, nh_private);

  ros::spin();

  return 0;
}
