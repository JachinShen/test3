#include "Serial.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <sstream>

Serial serial;

void armor_loc_callback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rm_serial");
    ros::NodeHandle node;

    serial.init();

    ros::Subscriber armor_loc_sub = node.subscribe("tpp/armor", 1, armor_loc_callback);
    ros::spin();

    return 0;
}

void armor_loc_callback(const std_msgs::String::ConstPtr& msg)
{
    int target_x, target_y, is_found;
    std::stringstream ss(msg->data.c_str());
    ss >> is_found >> target_x >> target_y;
    ROS_INFO_STREAM("Armor:" << (is_found ? 'Y' : 'N') << "x:" << target_x << "y:" << target_y);
    serial.sendTarget(target_x, target_y, is_found);
}
