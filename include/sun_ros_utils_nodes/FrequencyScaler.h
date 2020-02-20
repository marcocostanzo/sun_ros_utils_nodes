/*
    Frequency Scaler
    Copyright 2020 Università della Campania Luigi Vanvitelli
    Author: Marco Costanzo <marco.costanzo@unicampania.it>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SUN_ROS_UTILS_NODES_FREQUENCY_SCALER_H
#define SUN_ROS_UTILS_NODES_FREQUENCY_SCALER_H

#include "ros/ros.h"

namespace sun
{
template <class MSG_CLASS>
class FrequencyScalerNode
{
protected:
  // Protected members

  ros::NodeHandle nh_;

  ros::Timer timer_;

  // true if a new msg arrived
  bool msg_arrived_;

  // arrived msg
  MSG_CLASS msg_;

  // Publisher for the twist commands
  ros::Publisher pub_;

  // Command and measure wrench subscribers
  ros::Subscriber sub_;

public:
  // Constructor
  FrequencyScalerNode(const ros::NodeHandle& nh_public = ros::NodeHandle(),
                      const ros::NodeHandle& nh_private = ros::NodeHandle("~"))
    : nh_(nh_public)
  {
    msg_arrived_ = false;

    double rate;
    std::string sub_topic_str, pub_topic_str;
    get_ros_params(nh_private, rate, sub_topic_str, pub_topic_str);
    print_params(rate, sub_topic_str, pub_topic_str);

    // Subscriber
    sub_ = nh_.subscribe(sub_topic_str, 1, &FrequencyScalerNode::sub_cb_, this);

    // Publishers
    pub_ = nh_.advertise<MSG_CLASS>(pub_topic_str, 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &FrequencyScalerNode::timer_cb, this);
  }

  void get_ros_params(const ros::NodeHandle& nh_private, double& rate, std::string& sub_topic, std::string& pub_topic)
  {
    nh_private.getParam("rate", rate);
    nh_private.getParam("in_topic", sub_topic);
    nh_private.getParam("out_topic", pub_topic);
  }

  /*
    Print params
*/
  void print_params(double rate, const std::string& sub_topic, const std::string& pub_topic)
  {
    std::string output = "[FrequencyScaler]:\n";
    output += "\trate: " + std::to_string(rate) + "\n";
    output += "\tsub_topic: " + sub_topic + "\n";
    output += "\tpub_topic: " + pub_topic + "\n";

    ROS_INFO_STREAM(output);
  }

  void sub_cb_(const MSG_CLASS& msg)
  {
    msg_ = msg;
    msg_arrived_ = true;
  }

  void timer_cb(const ros::TimerEvent& event)
  {
    ros::spinOnce();  // force a spin here
    if (msg_arrived_)
    {
      pub_.publish(msg_);
      msg_arrived_ = false;
    }
  }

};  // end class

}  // end namespace sun

#endif