#include "TooN/TooN.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "toon_ros_msg_conversion/conversion.h"

namespace sun_ros_utils_nodes {

/**
Transfrorm a wrench i_i_w referred to pole of frame i and to axes of frame i
to the wrench f_b_w referred to pole of frame b and to axes of frame b
given the poses of the frames i and f both referred to a common frame b
*/
class RotateWrenchNodelet : public nodelet::Nodelet {

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

public:
  geometry_msgs::PoseStampedPtr b_P_i_;
  geometry_msgs::PoseStampedPtr b_P_f_;

  ros::Subscriber sub_i_i_wrench_;
  ros::Subscriber sub_b_pose_i_;
  ros::Subscriber sub_b_pose_f_;

  ros::Publisher i_f_wrench_pub_;

  std::vector<double> i_gains_vector;

  const std::vector<double> i_gains_default = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  ~RotateWrenchNodelet() = default;

  virtual void onInit() override {

    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle nh_private = getPrivateNodeHandle();

    std::string wrench_in_topic;
    nh_private.param("i_i_wrench_topic", wrench_in_topic,
                     std::string("wrench_in"));

    std::string wrench_out_topic;
    nh_private.param("i_f_wrench_topic", wrench_out_topic,
                     std::string("wrench_out"));

    nh_private.param("i_gains", i_gains_vector, i_gains_default);

    std::cout << "i_gains: ";
    for (int i = 0; i < 6; i++) {
      std::cout << i_gains_vector[i] << " ";
    }
    std::cout << std::endl;

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

    sub_i_i_wrench_ = nh.subscribe(wrench_in_topic, 1,
                                   &RotateWrenchNodelet::wrench_in_cb, this);

    if (!b_P_i_topic_str_.empty()) {
      boost::function<void(const geometry_msgs::PoseStampedPtr &)> f =
          boost::bind(&RotateWrenchNodelet::pose_cb, this, _1, &b_P_i_);
      sub_b_pose_i_ =
          nh.subscribe<geometry_msgs::PoseStamped>(b_P_i_topic_str_, 1, f);
    }
    if (!b_P_f_topic_str_.empty()) {
      boost::function<void(const geometry_msgs::PoseStampedPtr &)> f =
          boost::bind(&RotateWrenchNodelet::pose_cb, this, _1, &b_P_f_);
      sub_b_pose_f_ =
          nh.subscribe<geometry_msgs::PoseStamped>(b_P_f_topic_str_, 1, f);
    }

    i_f_wrench_pub_ =
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
      for (int i = 0; i < 6; i++) {
        i_i_wrench[i] *= i_gains_vector[i];
      }

      auto b_R_i = b_T_i.slice<0, 0, 3, 3>();
      auto b_R_f = b_T_f.slice<0, 0, 3, 3>();
      auto f_R_i = b_R_f.T() * b_R_i;

      TooN::Vector<6> i_f_wrench = rotateWrench(f_R_i, i_i_wrench);

      geometry_msgs::WrenchStampedPtr wrench_out(
          new geometry_msgs::WrenchStamped);

      wrench_out->wrench = sun::TooN2wrench(i_f_wrench);

      wrench_out->header.stamp = ros::Time::now();

      i_f_wrench_pub_.publish(wrench_out);
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