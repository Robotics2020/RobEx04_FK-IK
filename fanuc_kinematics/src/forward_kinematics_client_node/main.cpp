#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "fanuc_kinematics_msgs/ComputeFK.h"


int main(int argc, char** argv) {
    
    // Init node
    ros::init(argc, argv, "fanuc_FK_client");
    ros::NodeHandle nodeHandle;

    // Load robot model
    robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
    robot_model::RobotModelPtr robotModel = robotModelLoader.getModel();
    robot_state::RobotStatePtr robotState(new robot_state::RobotState(robotModel));
    std::string modelFrame = robotModel->getModelFrame();
    std::string modelGroup = robotModel->getSRDF()->getGroups()[0].name_;
    robot_state::JointModelGroup* jointModelGroup = robotModel->getJointModelGroup(modelGroup);
    std::vector<std::string> linkNames = jointModelGroup->getLinkModelNames();    

    // Get the service
    std::string serviceName;
    ros::param::get("FK_service", serviceName);
    ros::ServiceClient client = nodeHandle.serviceClient<fanuc_kinematics_msgs::ComputeFK>(serviceName);

    // Create request
    fanuc_kinematics_msgs::ComputeFK service;
    service.request.header.stamp = ros::Time::now();
    service.request.header.frame_id = modelFrame;
    service.request.fk_link_names = { linkNames.back() };
    moveit::core::robotStateToRobotStateMsg(*robotState, service.request.robot_state);

    // Post request
    ros::Rate rate = ros::Rate(0.1);
    while (true) {
        rate.sleep();
        if (client.call(service)) {
            continue;    
        } else {
            ROS_ERROR("Failed to call service %s", serviceName.c_str());
        }
    }

    return 0;
}