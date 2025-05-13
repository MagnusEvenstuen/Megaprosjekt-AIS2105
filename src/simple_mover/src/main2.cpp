#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create node with auto-param declaration
    auto node = std::make_shared<rclcpp::Node>(
        "hello_moveit2",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Declare parameters and defaults
    node->declare_parameter<double>("tx", -0.40349150880660745);
    node->declare_parameter<double>("ty", 0.21355450702058534);
    node->declare_parameter<double>("tz", 0.35585479647433027);
    node->declare_parameter<double>("rx", -0.9239217238827235);
    node->declare_parameter<double>("ry", 0.3795637577378466);
    node->declare_parameter<double>("rz", -0.0017038598898846053);
    node->declare_parameter<double>("rw", 0.04792805870236521);

    // Grab parameters into local vars
    double tx = node->get_parameter("tx").as_double();
    double ty = node->get_parameter("ty").as_double();
    double tz = node->get_parameter("tz").as_double();
    double rx = node->get_parameter("rx").as_double();
    double ry = node->get_parameter("ry").as_double();
    double rz = node->get_parameter("rz").as_double();
    double rw = node->get_parameter("rw").as_double();

    // Logger
    auto logger = rclcpp::get_logger("hello_moveit2");

    // MoveIt! interface (make sure your MoveIt! setup has a group called "ur_manipulator")
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_manipulator");

    // Build target pose from params
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = tx;
    target_pose.position.y = ty;
    target_pose.position.z = tz;
    target_pose.orientation.x = rx;
    target_pose.orientation.y = ry;
    target_pose.orientation.z = rz;
    target_pose.orientation.w = rw;

    RCLCPP_INFO(
      logger,
      "Setting target pose: pos(%.3f, %.3f, %.3f) ori(%.3f, %.3f, %.3f, %.3f)",
      tx, ty, tz, rx, ry, rz, rw
    );

    move_group.setPoseTarget(target_pose);

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group.plan(plan));

    if (success)
    {
        RCLCPP_INFO(logger, "Planning succeeded, executing...");
        move_group.execute(plan);
        RCLCPP_INFO(logger, "Execution complete!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    rclcpp::shutdown();
    return 0;
}
