#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

ros::Publisher pub;
double scale = 1.0;

void subCallback( sensor_msgs::JointState msg )
{
    
    for( int i=0; i<msg.position.size(); i++ ){
        msg.position[i] *= scale;
    }
    for( int i=0; i<msg.velocity.size(); i++ ){
        msg.velocity[i] *= scale;
    }
    for( int i=0; i<msg.effort.size(); i++ ){
        msg.effort[i] *= scale;
    }

    pub.publish(msg);

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "scale_joint_state");
    
    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");

    nh_private.param( "scale", scale, 1.0 );
    std::string topic_in_str, topic_out_str;
    nh_private.param( "topic_in", topic_in_str, std::string("joint_states") );
    nh_private.param( "topic_out", topic_out_str, std::string("joint_states/scale") );

    ros::Subscriber sub = nh.subscribe(topic_in_str, 1, subCallback);

    pub = nh.advertise<sensor_msgs::JointState>(topic_out_str, 1);
    
    ros::spin();

    return 0;
}