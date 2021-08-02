// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include "sun_ros_utils_nodes/wrench_difference_nodelet.h"

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(sun_ros_utils_nodes::WrenchDifferenceNodelet,
                       nodelet::Nodelet)
