#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <chrono>

struct BlockPosition {
    double x;
    double y;
    bool valid;
};

std::vector<BlockPosition> block_positions;
bool coordinates_received = false;

void coordinateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    block_positions.clear();
    for (size_t i = 0; i < msg->data.size(); i += 2) {
        BlockPosition pos;
        pos.x = msg->data[i];
        pos.y = msg->data[i + 1];
        pos.valid = (pos.x != -1000 && pos.y != -1000);
        block_positions.push_back(pos);
    }
    coordinates_received = true;
}

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

    // Create subscription to camera coordinates
    auto subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "distance_to_qube_center", 10, coordinateCallback);

    // Get initial pose parameters
    double rx = node->get_parameter("rx").as_double();
    double ry = node->get_parameter("ry").as_double();
    double rz = node->get_parameter("rz").as_double();
    double rw = node->get_parameter("rw").as_double();
    double home_x = node->get_parameter("tx").as_double();
    double home_y = node->get_parameter("ty").as_double();

    auto logger = rclcpp::get_logger("hello_moveit2");
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_manipulator");

    // Wait for camera coordinates
    RCLCPP_INFO(logger, "Waiting for block coordinates...");
    while (!coordinates_received && rclcpp::ok()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = rx;
    target_pose.orientation.y = ry;
    target_pose.orientation.z = rz;
    target_pose.orientation.w = rw;

    // Visit each valid block position
    for (const auto& block : block_positions) {
        if (!block.valid) continue;

        // Move to block position at safe height
        target_pose.position.x = block.x;
        target_pose.position.y = block.y;
        target_pose.position.z = 0.6;

        RCLCPP_INFO(logger, "Moving to position: (%.3f, %.3f, %.3f)", 
                   block.x, block.y, 0.6);
        sleep(3);

        if (moveToPosition(move_group, target_pose, logger)) {
            // Move down
            target_pose.position.z = 0.2;
            RCLCPP_INFO(logger, "Moving down to: (%.3f, %.3f, %.3f)", 
                       block.x, block.y, 0.2);
            sleep(3);
            
            if (moveToPosition(move_group, target_pose, logger)) {
                // Move back up
                target_pose.position.z = 0.6;
                RCLCPP_INFO(logger, "Moving up to: (%.3f, %.3f, %.3f)", 
                           block.x, block.y, 0.6);
                sleep(3);
                moveToPosition(move_group, target_pose, logger);
            }
        }
    }

    // Return to home position
    target_pose.position.x = home_x;
    target_pose.position.y = home_y;
    target_pose.position.z = 0.6;
    RCLCPP_INFO(logger, "Returning to home position");
    sleep(3);
    moveToPosition(move_group, target_pose, logger);

    rclcpp::shutdown();
    return 0;
}
