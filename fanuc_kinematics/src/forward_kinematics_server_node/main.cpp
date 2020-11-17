#include <ros/ros.h>
#include "fanuc_kinematics_msgs/ComputeFK.h"


bool compute_fk(fanuc_kinematics_msgs::ComputeFK::Request& request,
                fanuc_kinematics_msgs::ComputeFK::Response& response)
{
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "fanuc_FK_server");
    ros::NodeHandle nodeHandle;
    std::string serviceName;
    ros::param::get("FK_service", serviceName);
    ros::ServiceServer service = nodeHandle.advertiseService(serviceName, compute_fk);
    ros::spin();
    return 0;
}