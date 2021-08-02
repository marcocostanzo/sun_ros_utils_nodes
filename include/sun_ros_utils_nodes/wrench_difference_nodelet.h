#include "geometry_msgs/WrenchStamped.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"

namespace sun_ros_utils_nodes {

class WrenchDifferenceNodelet : public nodelet::Nodelet {

public:
  geometry_msgs::WrenchStampedPtr wrench1;
  geometry_msgs::WrenchStampedPtr wrench2;

  ros::Subscriber wrench1_sub;
  ros::Subscriber wrench2_sub;

  ros::Publisher out_pub;

  ~WrenchDifferenceNodelet() = default;

  virtual void onInit() override {

    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle nh_private = getPrivateNodeHandle();

    std::string wrench1_topic;
    std::string wrench2_topic;
    nh_private.param("wrench1_topic", wrench1_topic, std::string("wrench1"));
    nh_private.param("wrench2_topic", wrench2_topic, std::string("wrench2"));
    std::string out_topic;
    nh_private.param("out_topic", out_topic, std::string("out"));

    wrench1_sub = nh.subscribe(wrench1_topic, 1,
                               &WrenchDifferenceNodelet::wrench1_cb, this);

    wrench2_sub = nh.subscribe(wrench2_topic, 1,
                               &WrenchDifferenceNodelet::wrench2_cb, this);

    out_pub = nh.advertise<geometry_msgs::WrenchStamped>(out_topic, 1);
  }

  void wrench1_cb(const geometry_msgs::WrenchStampedPtr &msg) {
    wrench1 = msg;
    publishOut();
  }

  void wrench2_cb(const geometry_msgs::WrenchStampedPtr &msg) {
    wrench2 = msg;
    publishOut();
  }

  void publishOut() {
    if (wrench1 && wrench2) {
      geometry_msgs::WrenchStampedPtr out(new geometry_msgs::WrenchStamped);

      out->header.stamp = ros::Time::now();

      out->wrench.force.x = wrench1->wrench.force.x - wrench2->wrench.force.x;
      out->wrench.force.y = wrench1->wrench.force.y - wrench2->wrench.force.y;
      out->wrench.force.z = wrench1->wrench.force.z - wrench2->wrench.force.z;
      out->wrench.torque.x =
          wrench1->wrench.torque.x - wrench2->wrench.torque.x;
      out->wrench.torque.y =
          wrench1->wrench.torque.y - wrench2->wrench.torque.y;
      out->wrench.torque.z =
          wrench1->wrench.torque.z - wrench2->wrench.torque.z;

      out_pub.publish(out);
    }
  }
};

} // namespace sun_ros_utils_nodes