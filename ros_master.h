#ifndef ROS_MASTER_CLASSS
#define ROS_MASTER_CLASS 1 

#include <ros.h>


class ROS_master
{
    private:
        ros::NodeHandle* nh;

    public:
        ROS_master(ros::NodeHandle* nh);
        bool register_sub(ros::Subscriber* sub);
        bool register_pub(ros::Publisher* pub);

}

#endif