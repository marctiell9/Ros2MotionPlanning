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

// Gazebo messages/services in ROS2 (adjust if using gz-bridge or a different approach)
//#include "gazebo_msgs/srv/set_model_configuration.hpp"
//#include "std_srvs/srv/empty.hpp"

#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"  // For pausing the simulation

// KDL / URDF
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

// Your local headers (assuming you have migrated them to ROS2 as well)
#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"

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

// Main
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }
    
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create node
    auto node = rclcpp::Node::make_shared("kdl_ros2_control_node");
    //rclcpp::Client<example_interfaces::srv::SetEntityPose>::SharedPtr set_joint_positions_client; non so se ho bisogno di questa riga

    // Create subscriber
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    jointStateCallback
    );

    auto effort_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_controller/commands",
      10
    );
    auto error_pub = node->create_publisher<std_msgs::msg::Float64>("error", 10);

    // Create service clients (da capire come settare queste cose)    
    auto set_joint_positions_client = 
    node->create_client<ros_gz_interfaces::srv::SetEntityPose>(
    "/world/default/model/iiwa/set_joint_positions"
    );

    auto control_world_client =
    node->create_client<ros_gz_interfaces::srv::ControlWorld>(
    "/world/default/control"
    );
    

    // set the initial joint states with a position controller 
    /*
    auto position_publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/my_position_controller/commands", 10);
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {0.0, -1.57, 1.57, -1.2, 1.57, -1.57};
    position_publisher_->publish(msg);
    */

    // Prepare request for robot state, sono qui ora (da sistemare)
    /*auto robot_init_config = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    robot_init_config->state.name = "iiwa";
    robot_init_config->urdf_param_name = "robot_description";
    robot_init_config->joint_names = {
    "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3",
    "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6"
    };
    robot_init_config->joint_positions = {
    0.0, 1.57, -1.57, -1.2*/
  
    // Try to set robot state
    /*
    auto result_future = robot_set_state_client->async_send_request(robot_init_config);
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Robot state set.");
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Failed to set robot state.");
    }
    */

    // Create messages for torques
    std_msgs::msg::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg, err_msg;
    std_msgs::msg::Float64MultiArray cmd_msg; // utilizzato nel caso dei JointGroupEffortController
    cmd_msg.data.resize(7);


    // Spin or Wait until we have the first joint states
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
    double Error = 0.0;
    
    // Torques
    Eigen::VectorXd tau(robot.getNrJnts());
    tau.setZero();

    // Update robot (non viene messo)
    robot.update(jnt_pos, jnt_vel);

    // Init controller
    KDLController controller_(robot);

    // Initialize trajectory
    KDL::Frame init_cart_pose = robot.getEEFrame();
    Eigen::Vector3d init_position(init_cart_pose.p.data);
    Eigen::Vector3d end_position(init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z());

    // Plan trajectory
    double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, init_time_slot = 1.0, radius=0.10; 

    KDLPlanner planner(traj_duration, radius, acc_duration, init_position, end_position); // constructor with 5 arguments

    // KDLPlanner planner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
    // KDLPlanner planner(traj_duration, init_position, 0.10);
  
    int traj_type = 1; // <2 circular >=2 rectilinear
    trajectory_point p = planner.compute_trajectory(t, traj_type);

    double Kp = 100.0, Kd = 2.0 * std::sqrt(Kp);

    // Init trajectory
    KDL::Frame des_pose = KDL::Frame::Identity();
    KDL::Twist des_cart_vel = KDL::Twist::Zero();
    KDL::Twist des_cart_acc = KDL::Twist::Zero();
    des_pose.M = robot.getEEFrame().M;

    // We'll do a simple while loop with a fixed rate
    rclcpp::WallRate loop_rate(500); // 500 Hz
    auto begin = node->now();
    RCLCPP_INFO(node->get_logger(), "Starting control loop...");

    // Retrieve initial simulation time
    //rclcpp::Time begin = rclcpp::Clock().now(); capire se è questa la definizione giusta per il tempo o quello di sopra

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

      // Solve Inverse Kinematics and store results
      robot.getInverseKinematics(des_pose, des_cart_vel, des_cart_acc, qd, dqd, ddqd);

      // Joint Space Inverse Dynamics Control
      tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd, Error);

      // Fill messages
      /*
      tau1_msg.data = tau[0];
      tau2_msg.data = tau[1];
      tau3_msg.data = tau[2];
      tau4_msg.data = tau[3];
      tau5_msg.data = tau[4];
      tau6_msg.data = tau[5];
      */
      cmd_msg.data[0] = tau[0];
      cmd_msg.data[1] = tau[1];
      cmd_msg.data[2] = tau[2];
      cmd_msg.data[3] = tau[3];
      cmd_msg.data[4] = tau[4];
      cmd_msg.data[5] = tau[5];
      cmd_msg.data[6] = tau[6];

      err_msg.data  = Error;

      // Publish
      /*
      joint1_effort_pub->publish(tau1_msg);
      joint2_effort_pub->publish(tau2_msg);
      joint3_effort_pub->publish(tau3_msg);
      joint4_effort_pub->publish(tau4_msg);
      joint5_effort_pub->publish(tau5_msg);
      joint6_effort_pub->publish(tau6_msg);
      */
      error_pub->publish(err_msg);
      effort_pub->publish(cmd_msg);
    }

    // Spin callbacks (joint state updates, etc.) and wait
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  // Pause gazebo if desired (da sistemare)
  /*
  {
    auto pause_req = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    auto result_future = control_world_client->async_send_request(pause_req);
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Simulation paused.");
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Failed to pause simulation.");
    }
  }
  */

  // Shutdown
  rclcpp::shutdown();
  return 0;
}