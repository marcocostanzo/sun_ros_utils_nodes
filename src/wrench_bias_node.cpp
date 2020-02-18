/*
    ROS node to apply a bias to wrench stamped

    Copyright 2019 Università della Campania Luigi Vanvitelli

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
#include "geometry_msgs/WrenchStamped.h"
#include "std_srvs/Trigger.h"

using namespace std;

bool b_publish; //if can publish
bool b_wrench_in_arrived; //if wrench is arrived

int samples_to_use_for_bias;

geometry_msgs::WrenchStamped wrench_in;
geometry_msgs::WrenchStamped wrench_biased;
geometry_msgs::WrenchStamped wrench_bias;
ros::Publisher pub_wrench_bias;

//User fnc
void compute_bias();
void wait_wrench_measure();

void read_wrench_cb( const geometry_msgs::WrenchStamped::ConstPtr& msg  ){

    wrench_in = *msg;
    b_wrench_in_arrived = true;

    wrench_biased.header = wrench_in.header;

    wrench_biased.wrench.force.x = wrench_in.wrench.force.x - wrench_bias.wrench.force.x;
    wrench_biased.wrench.force.y = wrench_in.wrench.force.y - wrench_bias.wrench.force.y;
    wrench_biased.wrench.force.z = wrench_in.wrench.force.z - wrench_bias.wrench.force.z;
    wrench_biased.wrench.torque.x = wrench_in.wrench.torque.x - wrench_bias.wrench.torque.x;
    wrench_biased.wrench.torque.y = wrench_in.wrench.torque.y - wrench_bias.wrench.torque.y;
    wrench_biased.wrench.torque.z = wrench_in.wrench.torque.z - wrench_bias.wrench.torque.z;
	
    if(b_publish)
        pub_wrench_bias.publish(wrench_biased);

}

bool compute_bias_service_cb(std_srvs::Trigger::Request &request,
                                  std_srvs::Trigger::Response &response)
{
    compute_bias();
    response.message = "Bias Computed!";
    response.success = true;
    return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"wrench_bias");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    /**** PARAMS ****/
    string str_in_topic;
    nh_private.param("in_topic" , str_in_topic, string("wrench_in") );
    string str_out_topic;
    nh_private.param("out_topic" , str_out_topic, str_in_topic + string("_biased") );
    string str_bias_service;
    nh_private.param("bias_service" , str_bias_service, string("compute_bias") );
    nh_private.param("samples_to_use_for_bias" , samples_to_use_for_bias, 100 );
    /************************************/

    /*******INIT ROS PUB**********/
	pub_wrench_bias = nh_public.advertise<geometry_msgs::WrenchStamped>( str_out_topic, 1);
    /***************************/

    /*******INIT ROS SUB**********/
	ros::Subscriber sub_wrench = nh_public.subscribe( str_in_topic , 1, read_wrench_cb);
    /***************************/

    /*******INIT ROS SERVICES**********/
    ros::ServiceServer srv_compute_bias = nh_public.advertiseService(str_bias_service, compute_bias_service_cb);
    /***************************/

    //Compute the first bias!
    b_publish = false;
    compute_bias();
    b_publish = true;

    ros::spin();

    return 0;
}

void compute_bias()
{
    ROS_INFO("wrench_bias: Compute Bias...");
    //set bias to zero
    wrench_bias.wrench.force.x = 0.0;
    wrench_bias.wrench.force.y = 0.0;
    wrench_bias.wrench.force.z = 0.0;
    wrench_bias.wrench.torque.x = 0.0;
    wrench_bias.wrench.torque.y = 0.0;
    wrench_bias.wrench.torque.z = 0.0;

    //Compute bias
    for(int i=0; i<samples_to_use_for_bias; i++)
    {
        wait_wrench_measure();
        wrench_bias.wrench.force.x += wrench_in.wrench.force.x;
        wrench_bias.wrench.force.y += wrench_in.wrench.force.y;
        wrench_bias.wrench.force.z += wrench_in.wrench.force.z;
        wrench_bias.wrench.torque.x += wrench_in.wrench.torque.x;
        wrench_bias.wrench.torque.y += wrench_in.wrench.torque.y;
        wrench_bias.wrench.torque.z += wrench_in.wrench.torque.z;
    }
    wrench_bias.wrench.force.x /= samples_to_use_for_bias;
    wrench_bias.wrench.force.y /= samples_to_use_for_bias;
    wrench_bias.wrench.force.z /= samples_to_use_for_bias;
    wrench_bias.wrench.torque.x /= samples_to_use_for_bias;
    wrench_bias.wrench.torque.y /= samples_to_use_for_bias;
    wrench_bias.wrench.torque.z /= samples_to_use_for_bias;
    ROS_INFO("wrench_bias: Compute Bias DONE");
}

void wait_wrench_measure()
{   
    b_wrench_in_arrived = false;
    while(ros::ok() && !b_wrench_in_arrived)
        ros::spinOnce();
}