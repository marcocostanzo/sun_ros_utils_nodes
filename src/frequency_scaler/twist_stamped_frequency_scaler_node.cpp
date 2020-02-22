#include "geometry_msgs/TwistStamped.h"
#include "sun_ros_utils_nodes/FrequencyScaler.h"

int main(int argc, char *argv[])
{
  // ROS INIT - Use a coustom signinthandler
  ros::init(argc, argv, "twist_stamped_frequency_scaler");

  sun::FrequencyScalerNode<geometry_msgs::TwistStamped> frequency_scaler;

  ros::spin();

  return 0;
}
