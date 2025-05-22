#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

bool moveToPosition(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::Pose& target_pose,
    rclcpp::Logger& logger)
{
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group.plan(plan));

    if (success)
    {
        RCLCPP_INFO(logger, "Planning succeeded, executing...");
        move_group.execute(plan);
        RCLCPP_INFO(logger, "Execution complete!");
        return true;
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed!");
        return false;
    }
}

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create node with auto-param declaration from launch file
    auto node = std::make_shared<rclcpp::Node>(
        "hello_moveit2",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Grab parameters into local vars
    double tx = node->get_parameter("tx").as_double();
    double ty = node->get_parameter("ty").as_double();
    double tz = node->get_parameter("tz").as_double();
    double rx = node->get_parameter("rx").as_double();
    double ry = node->get_parameter("ry").as_double();
    double rz = node->get_parameter("rz").as_double();
    double rw = node->get_parameter("rw").as_double();

    auto logger = rclcpp::get_logger("hello_moveit2");
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_manipulator");

    // First position (higher z)
    geometry_msgs::msg::Pose first_pose;
    first_pose.position.x = tx;
    first_pose.position.y = ty;
    first_pose.position.z = 0.6;  // First height
    first_pose.orientation.x = rx;
    first_pose.orientation.y = ry;
    first_pose.orientation.z = rz;
    first_pose.orientation.w = rw;

    RCLCPP_INFO(
        logger,
        "Moving to first position: pos(%.3f, %.3f, %.3f)",
        tx, ty, 0.6
    );
    sleep(5);

    if (moveToPosition(move_group, first_pose, logger))
    {
        // Second position (lower z)
        geometry_msgs::msg::Pose second_pose = first_pose;
        second_pose.position.z = 0.6;  // Second height

        RCLCPP_INFO(
            logger,
            "Moving to second position: pos(%.3f, %.3f, %.3f)",
            tx, ty, 0.6
        );

        moveToPosition(move_group, second_pose, logger);
    }

    rclcpp::shutdown();
    return 0;
}
