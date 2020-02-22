
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace sun
{
void wrench_transform(const geometry_msgs::Wrench& data_in, geometry_msgs::Wrench& data_out,
                      const geometry_msgs::TransformStamped& transform, bool use_pole)
{
  // Apply the toration (doTransform<Vector3> applies the rotation only)
  tf2::doTransform(data_in, data_out, transform);

  if (use_pole)
  {  // apply the torque given by the pole
    tf2::Transform t;
    fromMsg(transform.transform, t);
    tf2::Vector3 rotated_force(data_out.force.x, data_out.force.y, data_out.force.z);
    tf2::Vector3 delta_torque = t.getOrigin().cross(rotated_force);
    data_out.torque.x += delta_torque[0];
    data_out.torque.y += delta_torque[1];
    data_out.torque.z += delta_torque[2];
  }
}
}

ros::Publisher pub;

// TF stuff
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::string target_fame;

bool use_pole;

void wrench_in_cb_(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  try
  {
    if (pub.getNumSubscribers()) //Compute only if there are subscribers
    {
      geometry_msgs::WrenchStamped out_msg;

      geometry_msgs::TransformStamped transform = tfBuffer->lookupTransform(
          target_fame, msg->header.frame_id,
          ros::Time(0),     // The time at which the value of the transform is desired. (0 will get the latest)
          ros::Duration(0)  // How long to block before failing
          );

      sun::wrench_transform(msg->wrench, out_msg.wrench, transform, use_pole);
      out_msg.header.stamp = msg->header.stamp;
      out_msg.header.frame_id = target_fame;

      pub.publish(out_msg);
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_THROTTLE(3.0, e.what());
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "transform_wrench");

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  std::string topic_wrench_in, topic_wrench_out;
  double tf_cache_time;

  nh_private.getParam("topic_wrench_in", topic_wrench_in);
  nh_private.getParam("topic_wrench_out", topic_wrench_out);
  nh_private.getParam("target_fame", target_fame);
  nh_private.param("use_pole", use_pole, false);
  nh_private.param("tf_cache_time", tf_cache_time, 1.0);

  ros::Subscriber sub = nh_public.subscribe(topic_wrench_in, 1, wrench_in_cb_);

  pub = nh_public.advertise<geometry_msgs::WrenchStamped>(topic_wrench_out, 1);

  tfBuffer = std::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::Duration(tf_cache_time)));

  tf2_ros::TransformListener tfListener(*tfBuffer);

  ros::spin();

  return 0;
}
