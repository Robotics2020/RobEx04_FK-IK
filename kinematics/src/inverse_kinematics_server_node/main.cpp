#include "ros/ros.h"

int main(int argc, char** argv) {
    // Init node
    ros::init(argc, argv, "IK_server");
    ros::NodeHandle nodeHandle;
    ros::Rate rate = ros::Rate(0.1);
    rate.sleep();
    return 0;
}