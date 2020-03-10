
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


ros::Publisher pub;

// TF stuff
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::string target_fame;

void pose_in_cb_(geometry_msgs::PoseStamped msg)
{
  try
  {
    if (pub.getNumSubscribers()) //Compute only if there are subscribers
    {

      geometry_msgs::TransformStamped transform = tfBuffer->lookupTransform(
          target_fame, msg.header.frame_id,
          ros::Time(0),     // The time at which the value of the transform is desired. (0 will get the latest)
          ros::Duration(0)  // How long to block before failing
          );
      tf2::doTransform(msg, msg, transform);

      pub.publish(msg);
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_THROTTLE(3.0, e.what());
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "transform_pose");

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  std::string topic_pose_in, topic_pose_out;
  double tf_cache_time;

  nh_private.getParam("topic_pose_in", topic_pose_in);
  nh_private.getParam("topic_pose_out", topic_pose_out);
  nh_private.getParam("target_fame", target_fame);
  nh_private.param("tf_cache_time", tf_cache_time, 1.0);

  ros::Subscriber sub = nh_public.subscribe(topic_pose_in, 1, pose_in_cb_);

  pub = nh_public.advertise<geometry_msgs::PoseStamped>(topic_pose_out, 1);

  tfBuffer = std::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::Duration(tf_cache_time)));

  tf2_ros::TransformListener tfListener(*tfBuffer);

  ros::spin();

  return 0;
}
