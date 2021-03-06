/*
    ROS node to differentiate float64 stamped

    Copyright 2021 Università della Campania Luigi Vanvitelli

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

#include "sun_ros_msgs/Float64Stamped.h"
#include "sun_systems_lib/TF/TF_DIFFERENTIATOR_2POLES.h"

using namespace std;

ros::Publisher pubFloatDiff;
sun_ros_msgs::Float64Stamped msgFloatDiff;

double value;

//==========TOPICs CALLBKs=========//
void readV(const sun_ros_msgs::Float64Stamped::ConstPtr &msg)
{
    if(isnan(msg->data) || isinf(msg->data))
    {
        return;
    }
    value = msg->data;
}

//====================================//

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "differentiate_float64stamped");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    /**** PARAMS ****/
    string str_in_topic = string("");
    nh_private.param("in_topic", str_in_topic, string("diff_in"));
    string str_out_topic = string("");
    nh_private.param("out_topic", str_out_topic, str_in_topic + string("_diff"));
    double poles_cut_freq;
    nh_private.param("poles_cut_freq", poles_cut_freq, 20.0);
    double Hz;
    nh_private.param("rate", Hz, 500.0);
    double gain;
    nh_private.param("gain", gain, 1.0);
    /************************************/

    /******INIT ROS MSGS**********/
    /********************/

    /*******INIT ROS PUB**********/
    //Float_filter pub
    pubFloatDiff = nh_public.advertise<sun_ros_msgs::Float64Stamped>(str_out_topic, 1);
    /***************************/

    /*******INIT ROS SUB**********/
    ros::Subscriber subFloat = nh_public.subscribe(str_in_topic, 1, readV);
    /***************************/

    /******INIT FILTER************/
    sun::TF_DIFFERENTIATOR_2POLES differentiator(1.0 / Hz, 1.0 / poles_cut_freq, gain);
    /***************************/

    /*============LOOP==============*/
    ros::Rate loop_rate(Hz);
    while (ros::ok())
    {

        msgFloatDiff.data = differentiator.apply(value);
        msgFloatDiff.header.stamp = ros::Time::now();
        pubFloatDiff.publish(msgFloatDiff);
        loop_rate.sleep();
        ros::spinOnce();
    }

    /*==============================*/

    return 0;
}