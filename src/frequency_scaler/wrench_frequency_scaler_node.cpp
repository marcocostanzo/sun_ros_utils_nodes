#include "sun_ros_utils_nodes/FrequencyScaler.h"
#include "geometry_msgs/WrenchStamped.h"

int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, "wrench_frequency_scaler");

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  sun::FrequencyScalerNode<geometry_msgs::WrenchStamped> frequency_scaler(nh_public, nh_private);

  ros::spin();

  return 0;
}
