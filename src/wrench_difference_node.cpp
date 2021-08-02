#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"

geometry_msgs::WrenchStampedPtr wrench1;
geometry_msgs::WrenchStampedPtr wrench2;

ros::Publisher out_pub;

void publishOut() {
  if (wrench1 && wrench2) {
    geometry_msgs::WrenchStampedPtr out(new geometry_msgs::WrenchStamped);

    out->header.stamp = ros::Time::now();

    out->wrench.force.x = wrench1->wrench.force.x - wrench2->wrench.force.x;
    out->wrench.force.y = wrench1->wrench.force.y - wrench2->wrench.force.y;
    out->wrench.force.z = wrench1->wrench.force.z - wrench2->wrench.force.z;
    out->wrench.torque.x = wrench1->wrench.torque.x - wrench2->wrench.torque.x;
    out->wrench.torque.y = wrench1->wrench.torque.y - wrench2->wrench.torque.y;
    out->wrench.torque.z = wrench1->wrench.torque.z - wrench2->wrench.torque.z;

    out_pub.publish(out);
  }
}

void wrench1_cb(const geometry_msgs::WrenchStampedPtr &msg) {
  wrench1 = msg;
  publishOut();
}

void wrench2_cb(const geometry_msgs::WrenchStampedPtr &msg) {
  wrench2 = msg;
  publishOut();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "wrench_difference");

  ros::NodeHandle nh_public;
  ros::NodeHandle nh_private("~");

  std::string wrench1_topic;
  std::string wrench2_topic;
  nh_private.param("wrench1_topic", wrench1_topic, std::string("wrench1"));
  nh_private.param("wrench2_topic", wrench2_topic, std::string("wrench2"));
  std::string out_topic;
  nh_private.param("out_topic", out_topic, std::string("out"));

  ros::Subscriber wrench1_sub =
      nh_public.subscribe(wrench1_topic, 1, wrench1_cb);

  ros::Subscriber wrench2_sub =
      nh_public.subscribe(wrench2_topic, 1, wrench2_cb);

  out_pub = nh_public.advertise<geometry_msgs::WrenchStamped>(out_topic, 1);

  ros::spin();

  return 0;
}
