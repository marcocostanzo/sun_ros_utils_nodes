/*
    ROS node to filter twist stamped

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

#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include "sun_systems_lib/TF/TF_FIRST_ORDER_FILTER.h"
#include "sun_systems_lib/TF/TF_MIMO_DIAGONAL.h"

using namespace TooN;
using namespace std;

ros::Publisher pubTwistFilter;
geometry_msgs::TwistStamped msgTwistFilter;

Vector<6> twist;

//==========TOPICs CALLBKs=========//
void readT(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  twist[0] = msg->twist.linear.x;
  twist[1] = msg->twist.linear.y;
  twist[2] = msg->twist.linear.z;
  twist[3] = msg->twist.angular.x;
  twist[4] = msg->twist.angular.y;
  twist[5] = msg->twist.angular.z;

  msgTwistFilter.header.frame_id = msg->header.frame_id;
}

//====================================//

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "filter_twist_stamped");

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public = ros::NodeHandle();

  /**** PARAMS ****/
  string str_in_topic = string("");
  nh_private.param("in_topic", str_in_topic, string("/twist"));
  string str_out_topic = string("");
  nh_private.param("out_topic", str_out_topic, str_in_topic + string("/filter"));
  double cut_freq;
  nh_private.param("cut_freq", cut_freq, 20.0);
  double Hz;
  nh_private.param("rate", Hz, 500.0);
  /************************************/

  /******INIT ROS MSGS**********/
  /********************/

  /*******INIT ROS PUB**********/
  pubTwistFilter = nh_public.advertise<geometry_msgs::TwistStamped>(str_out_topic, 1);
  /***************************/

  /*******INIT ROS SUB**********/
  ros::Subscriber subWrench = nh_public.subscribe(str_in_topic, 1, readT);
  /***************************/

  /******INIT FILTER************/
  sun::TF_MIMO_DIAGONAL filter(6, sun::TF_FIRST_ORDER_FILTER(cut_freq, 1.0 / Hz));
  /***************************/

  /*============LOOP==============*/
  ros::Rate loop_rate(Hz);
  while (ros::ok())
  {
    Vector<6> twist_filter = filter.apply(twist);

    // Fill msg
    msgTwistFilter.twist.linear.x = twist_filter[0];
    msgTwistFilter.twist.linear.y = twist_filter[1];
    msgTwistFilter.twist.linear.z = twist_filter[2];
    msgTwistFilter.twist.angular.x = twist_filter[3];
    msgTwistFilter.twist.angular.y = twist_filter[4];
    msgTwistFilter.twist.angular.z = twist_filter[5];

    msgTwistFilter.header.stamp = ros::Time::now();

    pubTwistFilter.publish(msgTwistFilter);
    loop_rate.sleep();
    ros::spinOnce();
  }

  /*==============================*/

  return 0;
}