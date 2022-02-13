#include <ros.h>

ROS_master::ROS_master(ros::NodeHandle* nh)
{
    this->nh = nh;
}

bool ROS_master::register_sub(ros::Subscriber* sub)
{
    this->(*nh).subscribe(*sub);
}

bool ROS_master::register_pub(ros::Publisher* pub)
{
    this->(*nh).advertise(pub);
}