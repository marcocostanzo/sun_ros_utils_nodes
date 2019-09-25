#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"

using namespace std;

ros::Publisher pub;
std::vector<double> scale;

void subCallback( geometry_msgs::WrenchStamped msg )
{
    msg.wrench.force.x *= scale[0];
    msg.wrench.force.y *= scale[1];
    msg.wrench.force.z *= scale[2];

    msg.wrench.force.x += 0.1;
    msg.wrench.force.y += 0.1;

    msg.wrench.torque.x *= scale[3];
    msg.wrench.torque.y *= scale[4];
    msg.wrench.torque.z *= scale[5];

    pub.publish(msg);

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "scale_joint_state");
    
    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");

    if(nh_private.hasParam("scale")){
        nh_private.getParam("scale", scale);
        if(scale.size() != 6){
            cout << "size scale wrench must be 6" << endl;
            exit(-1);
        }
    } else {
        cout << "no param scale wrench" << endl;
        exit(-1);
    }

    std::string topic_in_str, topic_out_str;
    nh_private.param( "topic_in", topic_in_str, std::string("wrench") );
    nh_private.param( "topic_out", topic_out_str, std::string("wrench/scale") );

    ros::Subscriber sub = nh.subscribe(topic_in_str, 1, subCallback);

    pub = nh.advertise<geometry_msgs::WrenchStamped>(topic_out_str, 1);
    
    ros::spin();

    return 0;
}