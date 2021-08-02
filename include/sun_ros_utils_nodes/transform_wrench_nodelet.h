#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace sun_ros_utils_nodes {

class TransformWrenchNodelet : public nodelet::Nodelet {

public:
  geometry_msgs::PoseStampedPtr b_P_e_;

  ros::Subscriber sub_wrench_in_;
  ros::Subscriber sub_pose_in_;

  ros::Publisher pub_out_;

  bool b_use_pole_ = false;

  ~TransformWrenchNodelet() = default;

  virtual void onInit() override {

    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle nh_private = getPrivateNodeHandle();

    nh_private.param("use_pole", b_use_pole_, false);

    std::string wrench_in_topic;
    nh_private.param("wrench_in_topic", wrench_in_topic,
                     std::string("wrench_in"));

    std::string wrench_out_topic;
    nh_private.param("wrench_out_topic", wrench_out_topic,
                     std::string("wrench_out"));

    std::string pose_in_topic;
    nh_private.param("pose_in_topic", pose_in_topic, std::string("wrench_in"));

    sub_wrench_in_ = nh.subscribe(wrench_in_topic, 1,
                                  &TransformWrenchNodelet::wrench_in_cb, this);
    sub_pose_in_ = nh.subscribe(pose_in_topic, 1,
                                &TransformWrenchNodelet::pose_in_cb, this);

    pub_out_ = nh.advertise<geometry_msgs::WrenchStamped>(wrench_out_topic, 1);
  }

  void pose_in_cb(const geometry_msgs::PoseStampedPtr &msg) { b_P_e_ = msg; }

  void wrench_in_cb(const geometry_msgs::WrenchStamped::ConstPtr &wrench_in) {

    if (b_P_e_) {
      geometry_msgs::TransformStamped transform;
      poseStampedToTransformStamped(*b_P_e_, transform);

      geometry_msgs::WrenchStampedPtr wrench_out(
          new geometry_msgs::WrenchStamped);

      wrench_transform(wrench_in->wrench, wrench_out->wrench, transform,
                       b_use_pole_);

      wrench_out->header.stamp = ros::Time::now();

      pub_out_.publish(wrench_out);
    }
  }

  static void wrench_transform(const geometry_msgs::Wrench &data_in,
                               geometry_msgs::Wrench &data_out,
                               const geometry_msgs::TransformStamped &transform,
                               bool use_pole) {
    // Apply the toration (doTransform<Vector3> applies the rotation only)
    tf2::doTransform(data_in, data_out, transform);

    if (use_pole) { // apply the torque given by the pole
      tf2::Transform t;
      fromMsg(transform.transform, t);
      tf2::Vector3 rotated_force(data_out.force.x, data_out.force.y,
                                 data_out.force.z);
      tf2::Vector3 delta_torque = t.getOrigin().cross(rotated_force);
      data_out.torque.x += delta_torque[0];
      data_out.torque.y += delta_torque[1];
      data_out.torque.z += delta_torque[2];
    }
  }

  static void
  poseStampedToTransformStamped(const geometry_msgs::PoseStamped &pose,
                                geometry_msgs::TransformStamped &transform,
                                const std::string &child_frame_id = "noframe") {
    transform.header = pose.header;
    transform.child_frame_id = child_frame_id;
    transform.transform.rotation = pose.pose.orientation;
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;
  }
};

} // namespace sun_ros_utils_nodes