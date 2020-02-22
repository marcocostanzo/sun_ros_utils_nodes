#include "geometry_msgs/WrenchStamped.h"
#include "sun_ros_utils_nodes/FrequencyScaler.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "wrench_stamped_frequency_scaler");

  sun::FrequencyScalerNode<geometry_msgs::WrenchStamped> frequency_scaler;

  ros::spin();

  return 0;
}
