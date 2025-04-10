#include "rclcpp/rclcpp.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include "kdl_ros_control/kdl_robot.h"

KDLRobot createRobot(const std::string & urdf_path, 
    const rclcpp::Node::SharedPtr & node)
{
// Parse URDF
urdf::Model model;
if (!model.initFile(urdf_path)) {
RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF file: %s", urdf_path.c_str());
}

// Build KDL tree
KDL::Tree robot_tree;
if (!kdl_parser::treeFromUrdfModel(model, robot_tree)) {
RCLCPP_ERROR(node->get_logger(), "Failed to construct KDL tree from URDF.");
}

// Create the robot
KDLRobot robot(robot_tree);
return robot;
}


// Main
int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("kdl_example_node");

    if (argc < 2) {
        RCLCPP_ERROR(node->get_logger(), "Usage: ros2 run <package> <executable> <path_to_urdf>");
        rclcpp::shutdown();
        return 1;
    }

    // 1) Create the robot from URDF
    std::string urdf_path = argv[1];
    KDLRobot robot = createRobot(urdf_path, node);

    int nrJnts = robot.getNrJnts();
    RCLCPP_INFO(node->get_logger(), "Robot has %d joints.", nrJnts);

    // 3) Update joint states
    std::vector<double> q(nrJnts, 1.0), qd(nrJnts, 0.0);
    robot.update(q, qd);
    
    // Just show them in a KDL array
    KDL::JntArray q_kdl(nrJnts);
    for (int i = 0; i < nrJnts; ++i) {
    q_kdl(i) = q[i];  // copy the joint positions in
    }

    for (unsigned int i = 0; i < q_kdl.data.size(); i++) {
    RCLCPP_INFO(node->get_logger(), "Joint %d position = %.3f", i, q_kdl(i));
    }
    
    // Specify an end-effector 
    robot.addEE(KDL::Frame::Identity());

    // Direct kinematics
    KDL::Frame F = robot.getEEFrame();

    std::ostringstream oss; //non so a cosa servono queste due righe 
    oss << F;  // let KDL operator<< format it
    RCLCPP_INFO(node->get_logger(), "Robot end-effector frame:\n%s", oss.str().c_str());


    // 6) Jacobian
    KDL::Jacobian J = robot.getEEJacobian();
    RCLCPP_INFO(node->get_logger(), "Jacobian rows=%d cols=%d:\n%s",
                J.rows(), J.columns(), 
                // Convert to string
                (std::stringstream() << J.data).str().c_str());


    // Inverse kinematics
    KDL::Frame F_target(F.M, F.p - KDL::Vector(0.0, 0.0, 0.1));
    KDL::JntArray q_result = robot.getInvKin(q_kdl, F_target);

    std::ostringstream oss2;
    oss2 << F_target;
    RCLCPP_INFO(node->get_logger(), "Desired end-effector frame:\n%s", oss2.str().c_str());

    RCLCPP_INFO(node->get_logger(), "Inverse Kinematics result:");
    for (unsigned int i = 0; i < q_result.data.size(); i++) {
    RCLCPP_INFO(node->get_logger(), "Joint %d position = %.3f", i, q_result(i));
    }

    return 0;
}
