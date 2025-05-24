#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>  // Add this include
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
    rclcpp::Logger& logger,
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_publisher)
{
    move_group.setPoseTarget(target_pose);

    geometry_msgs::msg::Point position;
    position.x = target_pose.position.x;
    position.y = target_pose.position.y;
    position_publisher->publish(position);

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

    auto position_publisher = node->create_publisher<geometry_msgs::msg::Point>(
    "target_position", 500);

    // Create subscription to camera coordinates
    auto subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "distance_to_qube_center", 500, coordinateCallback);

    // Get initial pose parameters
    double rx = node->get_parameter("rx").as_double();
    double ry = node->get_parameter("ry").as_double();
    double rz = node->get_parameter("rz").as_double();
    double rw = node->get_parameter("rw").as_double();
    double camera_x = node->get_parameter("tx").as_double();
    double camera_y = node->get_parameter("ty").as_double();
    double sleep_x = node->get_parameter("sleep_x").as_double();
    double sleep_y = node->get_parameter("sleep_y").as_double();
    double sleep_z = node->get_parameter("sleep_z").as_double();
    bool home_only = node->get_parameter("home").as_bool();

    auto logger = rclcpp::get_logger("hello_moveit2");
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_manipulator");

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = rx;
    target_pose.orientation.y = ry;
    target_pose.orientation.z = rz;
    target_pose.orientation.w = rw;

    if (home_only) {
        RCLCPP_INFO(logger, "Home-only mode: Moving directly to sleep position");
        target_pose.position.x = sleep_x;
        target_pose.position.y = sleep_y;
        target_pose.position.z = sleep_z;
        moveToPosition(move_group, target_pose, logger, position_publisher);
        rclcpp::shutdown();
        return 0;
    }

    // First move to camera position
    RCLCPP_INFO(logger, "Moving to camera position");
    target_pose.position.x = camera_x;
    target_pose.position.y = camera_y;
    target_pose.position.z = 0.6;
    moveToPosition(move_group, target_pose, logger, position_publisher);

    // Create publisher for triggering photo
    auto photo_trigger = node->create_publisher<std_msgs::msg::String>("take_photo", 500);
    
    // Wait a moment for the robot to stabilize
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Trigger photo
    auto msg = std_msgs::msg::String();
    msg.data = "photo";
    photo_trigger->publish(msg);
    RCLCPP_INFO(logger, "Triggered photo capture");

    // Wait for camera coordinates
    RCLCPP_INFO(logger, "Waiting for block coordinates...");
    while (!coordinates_received && rclcpp::ok()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    int conter = 0;
    bool notFound = false;
    for (const auto& block : block_positions){
        if (!block.valid){
            notFound = true;
            
            if (conter % 3 == 0){
                target_pose.position.x = camera_x + 0.2;
                target_pose.position.y = camera_y + 0.2;
            } else if (conter % 3 == 1){
                target_pose.position.x = camera_x - 0.2;
                target_pose.position.y = camera_y - 0.2;
            } else{
                target_pose.position.x = camera_x + 0.2;
                target_pose.position.y = camera_y - 0.2;
            }}
        else {
            notFound = false;
        }
    }

    while (notFound){
        for (const auto& block : block_positions){
            if (!block.valid){
                notFound = true;
                
                if (conter % 3 == 0){
                    target_pose.position.x = camera_x + 0.2;
                    target_pose.position.y = camera_y + 0.2;
                } else if (conter % 3 == 1){
                    target_pose.position.x = camera_x - 0.2;
                    target_pose.position.y = camera_y - 0.2;
                } else{
                    target_pose.position.x = camera_x + 0.2;
                    target_pose.position.y = camera_y - 0.2;
                }

                moveToPosition(move_group, target_pose, logger, position_publisher);
                
                sleep(2);
                auto msg = std_msgs::msg::String();
                msg.data = "photo";
                photo_trigger->publish(msg);
                RCLCPP_INFO(logger, "Triggered photo capture");
                
                break;
            } else {
                notFound = false;
            }
        }
        conter++;
    }

    if (!notFound){
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

            if (moveToPosition(move_group, target_pose, logger, position_publisher)) {
                // Move down
                target_pose.position.z = 0.2;
                RCLCPP_INFO(logger, "Moving down to: (%.3f, %.3f, %.3f)", 
                        block.x, block.y, 0.2);
                sleep(3);
                
                if (moveToPosition(move_group, target_pose, logger, position_publisher)) {
                    // Move back up
                    target_pose.position.z = 0.6;
                    RCLCPP_INFO(logger, "Moving up to: (%.3f, %.3f, %.3f)", 
                            block.x, block.y, 0.6);
                    sleep(3);
                    moveToPosition(move_group, target_pose, logger, position_publisher);
                }
            }
        }
    }

    // Return to Home position at the end
    target_pose.position.x = camera_x;
    target_pose.position.y = camera_y;
    target_pose.position.z = 0.6;
    RCLCPP_INFO(logger, "Returning to camera home position");
    sleep(3);
    moveToPosition(move_group, target_pose, logger, position_publisher);

    rclcpp::shutdown();
    return 0;
}
