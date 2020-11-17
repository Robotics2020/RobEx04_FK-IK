#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetPositionFK.h>
#include "fanuc_kinematics_msgs/ComputeFK.h"


template<class ServiceType>
void call_service(ros::ServiceClient& client, ServiceType& service, std::string modelFrame, std::string serviceName) {
    if (client.call(service)) {
        // Check if all FKs were computed and returned
        if (service.request.fk_link_names.size() != service.response.fk_link_names.size()) {
            ROS_WARN("Not all FKs computed!");
        }
        // Iterate over returned links
        for (std::size_t i = 0; i < service.response.fk_link_names.size(); ++i) {
            std::string link = service.response.fk_link_names[i];
            ROS_INFO_STREAM("******* Forward Kinematics from " << modelFrame << " to " << link << " *******\n" <<
                            service.response.pose_stamped[i].pose);
        }
    } else {
        ROS_ERROR("Failed to call service %s", serviceName.c_str());
    }
}


int main(int argc, char** argv) {
    // Parse aguments
    bool test;
    std::istringstream(argv[1]) >> std::boolalpha >> test;
    
    // Init node
    ros::init(argc, argv, "fanuc_FK_client");
    ros::NodeHandle nodeHandle;
    ros::Rate rate = ros::Rate(0.1);
    rate.sleep();

    // Load robot model
    robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
    robot_model::RobotModelPtr robotModel = robotModelLoader.getModel();
    robot_state::RobotStatePtr robotState(new robot_state::RobotState(robotModel));
    robotState->setToDefaultValues();
    std::string modelFrame = robotModel->getModelFrame();
    std::string modelGroup = robotModel->getSRDF()->getGroups()[0].name_;
    robot_state::JointModelGroup* jointModelGroup = robotModel->getJointModelGroup(modelGroup);
    std::vector<std::string> linkNames = jointModelGroup->getLinkModelNames();  
    const std::vector<std::string>& jointNames = jointModelGroup->getVariableNames();
    std::vector<double> jointValues;  

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

    // Create test request
    ros::ServiceClient test_client = nodeHandle.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
    moveit_msgs::GetPositionFK test_service;
    test_service.request.header.stamp = service.request.header.stamp;
    test_service.request.header.frame_id = service.request.header.frame_id;
    test_service.request.fk_link_names = service.request.fk_link_names;    

    // Loop
    while (true) {
        // Post request
        call_service(client, service, modelFrame, serviceName);
        // Compare with moveit_msgs/GetPositionFK Service
        if (test) {
            test_service.request.robot_state = service.request.robot_state;
            call_service(test_client, test_service, modelFrame, "moveit_msgs/GetPositionFK");
        }
        // Sleep
        rate.sleep();
        // Update request for new configuration
        robotState->setToRandomPositions(jointModelGroup);
        robotState->copyJointGroupPositions(jointModelGroup, jointValues);
        std::stringstream ss;
        ss << "******* New configuration: *******";
        for (std::size_t i = 0; i < jointNames.size(); ++i)
            ss << "\nJoint " << jointNames[i] << ": " << jointValues[i];
        ROS_INFO("%s", ss.str().c_str());
        ss.clear();
        moveit::core::robotStateToRobotStateMsg(*robotState, service.request.robot_state);
    }

    return 0;
}