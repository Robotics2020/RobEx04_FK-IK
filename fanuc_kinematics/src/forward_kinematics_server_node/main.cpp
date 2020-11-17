#include <ros/ros.h>


int main(int argc, char** argv) {
    
    // Init node
    ros::init(argc, argv, "fanuc_FK_server");
    ros::NodeHandle nodeHandle;
    
    // Start the main loop
    ros::Rate rate(0.1);
    while (nodeHandle.ok()) {
        rate.sleep();
    }

    return 0;
}