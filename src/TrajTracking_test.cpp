#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"

#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"

// for deactivating the position controller
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

// Global variables
std::vector<double> jnt_pos(7,0.0), jnt_vel(7,0.0), obj_pos(7,0.0),  obj_vel(7,0.0);
bool robot_state_available = false;

// Callback
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  robot_state_available = true;
  jnt_pos.clear();
  jnt_vel.clear();
  for (size_t i = 0; i < msg->position.size(); i++)
  {
    jnt_pos.push_back(msg->position[i]);
    jnt_vel.push_back(msg->velocity[i]);
  }
}

KDLRobot createRobot(const std::string &robot_string)
{
  urdf::Model model;
  if (!model.initFile(robot_string))
  {
    RCLCPP_ERROR(rclcpp::get_logger("kdl_ros2_control_node"), "Failed to parse URDF robot model.");
  }

  KDL::Tree robot_tree;
  if (!kdl_parser::treeFromUrdfModel(model, robot_tree))
  {
    RCLCPP_ERROR(rclcpp::get_logger("kdl_ros2_control_node"), "Failed to construct KDL tree.");
  }

  KDLRobot robot(robot_tree);
  return robot;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("kdl_ros2_control_node");

    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,jointStateCallback); // questo subscriber serve a leggere velocità e posizione giunti
    auto effort_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands",10);
    auto error_pub = node->create_publisher<std_msgs::msg::Float64>("error", 10);

    RCLCPP_INFO(node->get_logger(), "Waiting for robot joint states...");
    rclcpp::WallRate wait_rate(10); 
    while (rclcpp::ok() && !robot_state_available)
    {
    rclcpp::spin_some(node);
    wait_rate.sleep();
    }

    // Create robot
    KDLRobot robot = createRobot(argv[1]);
    robot.update(jnt_pos, jnt_vel);

    // Add an end-effector
    robot.addEE(KDL::Frame::Identity());

    // Jnt arrays
    KDL::JntArray qd(robot.getNrJnts()), dqd(robot.getNrJnts()), ddqd(robot.getNrJnts());
    dqd.data.setZero();
    ddqd.data.setZero();

    // Error variable
    double Error;
    
    // Torques
    Eigen::VectorXd tau(robot.getNrJnts());
    tau.setZero();

    // Init controller
    KDLController controller_(robot);

    // Initialize trajectory
    KDL::Frame init_cart_pose = robot.getEEFrame();
    RCLCPP_INFO(node->get_logger(), "Cart pose x=%.3f y=%.3f z=%.3f: ", init_cart_pose.p.x(), init_cart_pose.p.y(), init_cart_pose.p.z());

    ///////////// TEST
  
    // setting the initial state with the position controller
    auto position_publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {0.0, 1.57, -1.57, -1.2, 1.57, -1.57, -0.37};
    position_publisher_->publish(msg);
    
    RCLCPP_INFO(node->get_logger(), "Waiting for robot to move...");
    rclcpp::WallRate rate(50);  // 50 Hz
    for (int i = 0; i < 100; ++i)  // wait some seconds
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    robot.update(jnt_pos, jnt_vel);
    robot.addEE(KDL::Frame::Identity()); 
    init_cart_pose = robot.getEEFrame();
    Eigen::Vector3d init_position(init_cart_pose.p.data);
    Eigen::Vector3d end_position(init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z());
    RCLCPP_INFO(node->get_logger(), "Cart position x=%.3f y=%.3f z=%.3f: ", init_cart_pose.p.x(), init_cart_pose.p.y(), init_cart_pose.p.z());
    RCLCPP_INFO(node->get_logger(), "Rotation matrix:");
    RCLCPP_INFO(node->get_logger(), "[%.3f %.3f %.3f]", 
                init_cart_pose.M(0,0), init_cart_pose.M(0,1), init_cart_pose.M(0,2));
    RCLCPP_INFO(node->get_logger(), "[%.3f %.3f %.3f]", 
                init_cart_pose.M(1,0), init_cart_pose.M(1,1), init_cart_pose.M(1,2));
    RCLCPP_INFO(node->get_logger(), "[%.3f %.3f %.3f]", 
                init_cart_pose.M(2,0), init_cart_pose.M(2,1), init_cart_pose.M(2,2));

    /////////////////////////////////////////////////////////////////////////////////// SWITCH CONTROLLERS
    
    auto client = node->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->deactivate_controllers = {"position_controller"};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    if (!client->wait_for_service(std::chrono::seconds(5))) return 0;
    auto result_future = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, result_future);

    // Load the controller
    auto load_client = node->create_client<controller_manager_msgs::srv::LoadController>(
      "/controller_manager/load_controller");
    auto load_request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
    load_request->name = "effort_controller";

    if (!load_client->wait_for_service(std::chrono::seconds(5))) return 0;
    auto load_result = load_client->async_send_request(load_request);
    rclcpp::spin_until_future_complete(node, load_result);

    // Configure the controller
    auto configure_client = node->create_client<controller_manager_msgs::srv::ConfigureController>(
        "/controller_manager/configure_controller");
    auto configure_request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
    configure_request->name = "effort_controller";

    if (!configure_client->wait_for_service(std::chrono::seconds(5))) return 0;
    auto configure_result = configure_client->async_send_request(configure_request);
    rclcpp::spin_until_future_complete(node, configure_result);

    // Activate the controller
    auto switch_client = node->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");
    auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_request->activate_controllers = {"effort_controller"};
    switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    if (!switch_client->wait_for_service(std::chrono::seconds(5))) return 0;
    auto switch_result = switch_client->async_send_request(switch_request);
    rclcpp::spin_until_future_complete(node, switch_result);

    ////////////////////////////////////////////////////////////////////////////////    
    // Plan trajectory
    double traj_duration = 5, acc_duration = 1, t = 0.0, init_time_slot = 1.0, radius=0.10; 
    KDLPlanner planner(traj_duration, radius, acc_duration, init_position, end_position); // constructor with 5 arguments
    int traj_type = 2; // <2 circular >=2 rectilinear
    trajectory_point p = planner.compute_trajectory(t, traj_type);

    double Kp = 10.0, Kd = std::sqrt(Kp);

    // Init trajectory
    KDL::Frame des_pose = KDL::Frame::Identity();
    KDL::Twist des_cart_vel = KDL::Twist::Zero();
    KDL::Twist des_cart_acc = KDL::Twist::Zero();
    des_pose.M = robot.getEEFrame().M; // desired rotation matrix

    /*
    ////////////////////////////// test effort_pub
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.resize(7);  // Allocate space for 7 elements
    rclcpp::WallRate loop_rate(1);  // 1 Hz = once per second

    while (rclcpp::ok())
    {
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = {100, 200, 5, 0, 4000, 0, 0};  // Example torque values

        effort_pub->publish(cmd_msg);
        RCLCPP_INFO(node->get_logger(), "Published effort command");

        rclcpp::spin_some(node);
        loop_rate.sleep();  // Wait for next iteration
    }
    /////////////////////////////
    */

    //////////// WHILE CONTROL LOOP 
    // We'll do a simple while loop with a fixed rate
    // Create messages for torques
    std_msgs::msg::Float64 err_msg;
    std_msgs::msg::Float64MultiArray cmd_msg; 
    cmd_msg.data.resize(7);
    rclcpp::WallRate loop_rate(500); // 500 Hz
    auto begin = node->now();
    RCLCPP_INFO(node->get_logger(), "Starting control loop...");
    while (rclcpp::ok())
    {
    double elapsed = (node->now() - begin).seconds();

    if (elapsed > 2 * traj_duration + init_time_slot)
    {
      RCLCPP_INFO(node->get_logger(), "Trajectory ended, exiting control loop.");
      break;
    }
    
    if (robot_state_available)
    {
      // Update the robot with the latest joint states
      robot.update(jnt_pos, jnt_vel);

      // Compute time
      t = elapsed;
      RCLCPP_DEBUG(node->get_logger(), "Time: %f", t);
      des_cart_vel = KDL::Twist::Zero();
      des_cart_acc = KDL::Twist::Zero();

      if (t <= init_time_slot)
      {
        // Wait a bit
        p = planner.compute_trajectory(0.0, traj_type);
      }
      else if (t > init_time_slot && t <= traj_duration + init_time_slot)
      {
        p = planner.compute_trajectory(t - init_time_slot, traj_type);
        des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]), KDL::Vector::Zero());
        des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]), KDL::Vector::Zero());
      }
      else
      {
        RCLCPP_INFO(node->get_logger(), "Trajectory terminated or out of range.");
        break;
      }

      des_pose.p = KDL::Vector(p.pos[0], p.pos[1], p.pos[2]);

      // Inverse Kinematics
      qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];
      qd = robot.getInvKin(qd, des_pose); 
      //RCLCPP_INFO(node->get_logger(), "Inverse kinematics results: %f", qd.data[0]);

      // Solve Inverse Kinematics and store results
       robot.getInverseKinematics(des_pose, des_cart_vel, des_cart_acc, qd, dqd, ddqd);

      // Joint Space Inverse Dynamics Control
      tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd, Error);

      /////////// Send commands to the position controller
      //msg.data = {qd.data[0], qd.data[1], qd.data[2], qd.data[3], qd.data[4], qd.data[5], qd.data[6]};
      //position_publisher_->publish(msg);
      ///////////
      
      cmd_msg.data[0] = tau[0];
      cmd_msg.data[1] = tau[1];
      cmd_msg.data[2] = tau[2];
      cmd_msg.data[3] = tau[3];
      cmd_msg.data[4] = tau[4];
      cmd_msg.data[5] = tau[5];
      cmd_msg.data[6] = tau[6];

      err_msg.data  = Error;

      error_pub->publish(err_msg);
      effort_pub->publish(cmd_msg);  
          
    }

    // Spin callbacks (joint state updates, etc.) and wait
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
////////////////////////////////////////////////////////////////////////////////////
// the robot should stay still in the last planned position
  while (rclcpp::ok())
    {
      robot.update(jnt_pos, jnt_vel);

      // Inverse Kinematics
      qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];
      qd = robot.getInvKin(qd, des_pose); 
      // Solve Inverse Kinematics and store results
       robot.getInverseKinematics(des_pose, des_cart_vel, des_cart_acc, qd, dqd, ddqd);
      // Joint Space Inverse Dynamics Control
      tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd, Error);
        
      cmd_msg.data[0] = tau[0];
      cmd_msg.data[1] = tau[1];
      cmd_msg.data[2] = tau[2];
      cmd_msg.data[3] = tau[3];
      cmd_msg.data[4] = tau[4];
      cmd_msg.data[5] = tau[5];
      cmd_msg.data[6] = tau[6];

      err_msg.data  = Error;

      error_pub->publish(err_msg);
      effort_pub->publish(cmd_msg);      
  
      // Spin callbacks (joint state updates, etc.) and wait
      rclcpp::spin_some(node);
      loop_rate.sleep();
  }
    rclcpp::shutdown();
    return 0;
}
