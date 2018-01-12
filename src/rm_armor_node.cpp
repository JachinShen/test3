#include <iostream>
#include <vector>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Armor.h"
//#include "Serial.h"
#include "std_msgs/String.h"
#include <ros/ros.h>

#include <fstream>

/* global publisher */
ros::Publisher armor_loc_pub;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rm_challenge_camera");
    ros::NodeHandle node;

    armor_loc_pub = node.advertise<std_msgs::String>("tpp/armor", 1);
#if PLATFORM == MANIFOLD
    cv::VideoCapture cap(0);
#else
    cv::VideoCapture cap("/home/jachinshen/视频/Robo/train.avi");
#endif
    if (!cap.isOpened()) {
        ROS_INFO("camera not open");
        return -1;
    }
    cv::Mat src;
    cap.read(src);
    //cv::namedWindow("frame", 1);
    //cv::imshow("frame", src);
    //cv::waitKey(0);
    Armor armor;
    armor.setDraw(SHOW_ALL);
    armor.init(src);
    //Serial serial;
    //serial.init();
    while (ros::ok()) {
        cap >> src;
        if (src.empty())
            break;
        if (armor.isFound()) {
            armor.track(src);
        } else {
            armor.explore(src);
        }
        std::stringstream ss;
        std_msgs::String armor_msg;
        ss << armor.isFound()
           << " " << armor.getTargetX()
           << " " << armor.getTargetY();
        cout << armor.isFound()
             << " " << armor.getTargetX()
             << " " << armor.getTargetY() << endl;

        armor_msg.data = ss.str();
        armor_loc_pub.publish(armor_msg);
        //serial.sendTarget(armor.getTargetX(), armor.getTargetY(), armor.isFound());

        cv::imshow("frame", src);
        cv::waitKey(0);
        ros::spinOnce();
    }
    cap.release();
}
