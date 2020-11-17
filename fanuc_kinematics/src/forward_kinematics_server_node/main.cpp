#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "fanuc_kinematics_msgs/ComputeFK.h"


bool compute_fk(fanuc_kinematics_msgs::ComputeFK::Request& request, fanuc_kinematics_msgs::ComputeFK::Response& response) {    
    // Load robot model
    robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
    robot_model::RobotModelPtr robotModel = robotModelLoader.getModel();
    robot_state::RobotStatePtr robotState(new robot_state::RobotState(robotModel));

    // Get request parameters
    std::vector<std::string> links = request.fk_link_names;
    bool success = moveit::core::robotStateMsgToRobotState(request.robot_state, *robotState);
    if (!success) return false;

    // Iterate over links
    for(std::size_t i = 0; i < links.size(); ++i) {
        std::string link = links[i];
        // Compute FK for each link
        const Eigen::Isometry3d& endEffectorState = robotState->getGlobalLinkTransform(link);
        ROS_INFO_STREAM("TEST Translation: \n" << endEffectorState.translation() << "\n");
        ROS_INFO_STREAM("TEST Rotation: \n" << endEffectorState.rotation() << "\n");
    }

    return true;
}


int main(int argc, char** argv) {
    // Init node
    ros::init(argc, argv, "fanuc_FK_server");
    ros::NodeHandle nodeHandle;
    
    // Start the service
    std::string serviceName;
    ros::param::get("FK_service", serviceName);
    ros::ServiceServer service = nodeHandle.advertiseService(serviceName, compute_fk);
    
    // Loop
    ros::spin();
    return 0;
}