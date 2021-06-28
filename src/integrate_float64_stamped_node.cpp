
#include "ros/ros.h"

#include "sun_ros_msgs/Float64Stamped.h"
#include "std_srvs/Trigger.h"

#include "sun_systems_lib/TF/TF_INTEGRATOR.h"

using namespace std;

std::unique_ptr<sun::TF_INTEGRATOR> integrator;

ros::Publisher pubFloatFilter;
sun_ros_msgs::Float64Stamped msgFloatFilter;

double data_;
double input_saturation;

/*Set Running callback*/
bool reset_callbk(std_srvs::Trigger::Request &req,
                  std_srvs::Trigger::Response &res)
{
    ROS_INFO_STREAM("RESETTING INTEGRATOR...");
    integrator->reset();
    res.success = true;
    ROS_INFO_STREAM("INTEGRATOR RESETTED");
    return true;
}

//==========TOPICs CALLBKs=========//
void readF(const sun_ros_msgs::Float64Stamped::ConstPtr &msg)
{

    if (isnan(msg->data) || isinf(msg->data) || fabs(msg->data) <= input_saturation)
    {
        data_ = 0;
    }
    else
    {
        data_ = msg->data;
    }
    // std::cout << "***********\ninput_saturation:" << input_saturation << "\nmsg: " << msg->data << "\ndata_:" << data_ << std::endl;
}

//====================================//

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "filter_float64stamped");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    /**** PARAMS ****/
    string str_in_topic = string("");
    nh_private.param("in_topic", str_in_topic, string("in_int"));
    string str_out_topic = string("");
    nh_private.param("out_topic", str_out_topic, str_in_topic + string("out_int"));
    double gain;
    nh_private.param("gain", gain, 1.0);
    double Hz;
    nh_private.param("rate", Hz, 500.0);
    nh_private.param("input_saturation", input_saturation, std::numeric_limits<double>::infinity());
    string str_reset_srv;
    nh_private.param("reset_srv", str_reset_srv, string("reset"));
    /************************************/

    /******INIT ROS MSGS**********/
    /********************/

    /*******INIT ROS PUB**********/
    //Float_filter pub
    pubFloatFilter = nh_public.advertise<sun_ros_msgs::Float64Stamped>(str_out_topic, 1);
    /***************************/

    /*******INIT ROS SUB**********/
    ros::Subscriber subFloat = nh_public.subscribe(str_in_topic, 1, readF);
    /***************************/

    /*******INIT ROS SERV**********/
    ros::ServiceServer servReset = nh_public.advertiseService(str_reset_srv, reset_callbk);
    /***************************/

    /******INIT FILTER************/
    integrator = std::unique_ptr<sun::TF_INTEGRATOR>(new sun::TF_INTEGRATOR(1.0 / Hz, gain));
    /***************************/

    /*============LOOP==============*/
    ros::Rate loop_rate(Hz);
    while (ros::ok())
    {

        msgFloatFilter.data = integrator->apply(data_);
        msgFloatFilter.header.stamp = ros::Time::now();
        pubFloatFilter.publish(msgFloatFilter);
        loop_rate.sleep();
        ros::spinOnce();
    }

    /*==============================*/

    return 0;
}