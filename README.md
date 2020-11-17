# RobEx4_InvKin

1. Implement a service server that computes the direct kinematics of a robot and a service client that uses this service and prints the solution to stdout
2. Verify the result by comparing it with the service /compute_fk of the move_group node
3. Implement an action server that computes all the inverse kinematic solutions of a robot (one by one) and an action client that uses this action and prints the solutions to stdout (one by one, as their are received). The action server does not send the same solution twice and stops when all solutions have been found. At that time, they are returned all together
4. Repeat the experiment at point 3 by neglecting joint limits
5. Visualize the IK solutions in RViz

## Point 1

> Implement a service server that computes the direct kinematics of a robot and a service client that uses this service and prints the solution to stdout

* The service is called [ComputeFK.srv](https://github.com/Robotics2020/RobEx04_InvKin/blob/master/fanuc_kinematics_msgs/srv/ComputeFK.srv). It has the same format as [moveit_msgs/GetPositionFK Service](http://docs.ros.org/en/melodic/api/moveit_msgs/html/srv/GetPositionFK.html).
* The service server is implemented in the [forward_kinematics_server_node](https://github.com/Robotics2020/RobEx04_InvKin/blob/master/fanuc_kinematics/src/forward_kinematics_server_node/main.cpp)
* The service client is implemented in the [forward_kinematics_client_node](https://github.com/Robotics2020/RobEx04_InvKin/tree/master/fanuc_kinematics/src/forward_kinematics_client_node/main.cpp)

## Point 2

> Verify the result by comparing it with the service /compute_fk of the move_group node

## Point 3

> Implement an action server that computes all the inverse kinematic solutions of a robot (one by one) and an action client that uses this action and prints the solutions to stdout (one by one, as their are received). The action server does not send the same solution twice and stops when all solutions have been found. At that time, they are returned all together

## Point 4

> Repeat the experiment at point 3 by neglecting joint limits

## Point 5

> Visualize the IK solutions in RViz

## Usage

### Forward Kinematics

To compute FK without comparing with [moveit_msgs/GetPositionFK Service](http://docs.ros.org/en/melodic/api/moveit_msgs/html/srv/GetPositionFK.html), run

```bash
roslaunch kinematics fk.launch
```

or

```bash
roslaunch kinematics fk.launch test:=false
```

To compute FK and compare it with [moveit_msgs/GetPositionFK Service](http://docs.ros.org/en/melodic/api/moveit_msgs/html/srv/GetPositionFK.html), run

```bash
roslaunch kinematics fk.launch test:=true
```

### Inverse Kinematics
