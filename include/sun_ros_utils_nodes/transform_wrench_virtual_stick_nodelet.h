#include "TooN/TooN.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "toon_ros_msg_conversion/conversion.h"

namespace sun_ros_utils_nodes {

bool getPoseFromParam(ros::NodeHandle &nh, geometry_msgs::Pose &out,
                      const std::string &param_name) {
  std::map<std::string, double> map_s;
  if (nh.hasParam(param_name)) {
    try {
      nh.getParam(param_name, map_s);
      out.position.x = map_s["position_x"];
      out.position.y = map_s["position_y"];
      out.position.z = map_s["position_z"];
      out.orientation.w = map_s["orientation_w"];
      out.orientation.x = map_s["orientation_x"];
      out.orientation.y = map_s["orientation_y"];
      out.orientation.z = map_s["orientation_z"];

      return true;
    } catch (...) {
      ROS_ERROR_STREAM("INVALID PARAM " << param_name);
      return false;
    }
  } else {
    return false;
  }
}

/**
Transfrorm a wrench i_i_w referred to pole of frame i and to axes of frame i
to the wrench f_b_w referred to pole of frame b and to axes of frame b
given the poses of the frames i and f both referred to a common frame b
*/
class TransformWrenchVirtualStickNodelet : public nodelet::Nodelet {

public:
  geometry_msgs::PoseStampedPtr b_P_i_;
  geometry_msgs::PoseStampedPtr b_P_f_;

  ros::Subscriber sub_i_i_wrench_;
  ros::Subscriber sub_b_pose_i_;
  ros::Subscriber sub_b_pose_f_;

  ros::Publisher f_b_wrench_pub_;

  bool b_use_pole_ = false;

  ~TransformWrenchVirtualStickNodelet() = default;

  virtual void onInit() override {

    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle nh_private = getPrivateNodeHandle();

    std::string wrench_in_topic;
    nh_private.param("i_i_wrench_topic", wrench_in_topic,
                     std::string("wrench_in"));

    std::string wrench_out_topic;
    nh_private.param("f_b_wrench_topic", wrench_out_topic,
                     std::string("wrench_out"));

    std::string b_P_i_topic_str_;
    nh_private.param("b_pose_i_topic", b_P_i_topic_str_, std::string(""));
    std::string b_P_f_topic_str_;
    nh_private.param("b_pose_f_topic", b_P_f_topic_str_, std::string(""));

    geometry_msgs::Pose b_P_i_initial_;
    if (getPoseFromParam(nh_private, b_P_i_initial_, "b_pose_i_initial")) {
      b_P_i_ = geometry_msgs::PoseStampedPtr(new geometry_msgs::PoseStamped);
      b_P_i_->header.stamp = ros::Time::now();
      b_P_i_->pose = b_P_i_initial_;
    }

    geometry_msgs::Pose b_P_f_initial_;
    if (getPoseFromParam(nh_private, b_P_f_initial_, "b_pose_f_initial")) {
      b_P_f_ = geometry_msgs::PoseStampedPtr(new geometry_msgs::PoseStamped);
      b_P_f_->header.stamp = ros::Time::now();
      b_P_f_->pose = b_P_f_initial_;
    }

    sub_i_i_wrench_ =
        nh.subscribe(wrench_in_topic, 1,
                     &TransformWrenchVirtualStickNodelet::wrench_in_cb, this);

    if (!b_P_i_topic_str_.empty()) {
      boost::function<void(const geometry_msgs::PoseStampedPtr &)> f =
          boost::bind(&TransformWrenchVirtualStickNodelet::pose_cb, this, _1,
                      &b_P_i_);
      sub_b_pose_i_ =
          nh.subscribe<geometry_msgs::PoseStamped>(b_P_i_topic_str_, 1, f);
    }
    if (!b_P_f_topic_str_.empty()) {
      boost::function<void(const geometry_msgs::PoseStampedPtr &)> f =
          boost::bind(&TransformWrenchVirtualStickNodelet::pose_cb, this, _1,
                      &b_P_f_);
      sub_b_pose_f_ =
          nh.subscribe<geometry_msgs::PoseStamped>(b_P_f_topic_str_, 1, f);
    }

    f_b_wrench_pub_ =
        nh.advertise<geometry_msgs::WrenchStamped>(wrench_out_topic, 1);
  }

  void pose_cb(const geometry_msgs::PoseStampedPtr &msg,
               geometry_msgs::PoseStampedPtr *out_ptr) {
    *out_ptr = msg;
  }

  void wrench_in_cb(const geometry_msgs::WrenchStamped::ConstPtr &wrench_in) {

    if (b_P_i_ && b_P_f_) {
      TooN::Matrix<4, 4> b_T_i = sun::pose2TooN(b_P_i_->pose);
      TooN::Matrix<4, 4> b_T_f = sun::pose2TooN(b_P_f_->pose);
      TooN::Vector<6> i_i_wrench = sun::wrench2TooN(wrench_in->wrench);

      auto b_R_i = b_T_i.slice<0, 0, 3, 3>();
      auto b_r_i = b_T_i[3].slice<0, 3>();
      auto b_r_f = b_T_f[3].slice<0, 3>();
      auto b_r_f_i = b_r_f - b_r_i;

      TooN::Vector<6> i_b_wrench = rotateWrench(b_R_i, i_i_wrench);

      TooN::Vector<6> f_b_wrench = i_b_wrench;
      f_b_wrench.slice<3, 3>() += (b_r_f_i ^ i_b_wrench.slice<0, 3>());

      geometry_msgs::WrenchStampedPtr wrench_out(
          new geometry_msgs::WrenchStamped);

      wrench_out->wrench = sun::TooN2wrench(f_b_wrench);

      wrench_out->header.stamp = ros::Time::now();

      f_b_wrench_pub_.publish(wrench_out);
    }
  }

  static TooN::Vector<6> rotateWrench(const TooN::Matrix<3, 3> &b_R_e,
                                      const TooN::Vector<6> e_wrench) {
    TooN::Vector<6> b_wrench;
    b_wrench.slice<0, 3>() = b_R_e * e_wrench.slice<0, 3>();
    b_wrench.slice<3, 3>() = b_R_e * e_wrench.slice<3, 3>();
    return b_wrench;
  }
};

} // namespace sun_ros_utils_nodes