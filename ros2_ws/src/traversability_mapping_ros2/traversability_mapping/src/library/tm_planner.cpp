#include <pluginlib/class_list_macros.hpp>
#include "planner/tm_planner.h"

// Register this planner as a GlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(tm_planner::TMPlanner, nav2_core::GlobalPlanner)

namespace tm_planner {

void TMPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;

    // Initialize subscriptions
    subPath = node_->create_subscription<nav_msgs::msg::Path>(
        "/global_path", 5, 
        std::bind(&TMPlanner::pathHandler, this, std::placeholders::_1));
    
    // Changed from /prm_goal to /goal_pose to match RViz2
    pubGoal = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 5);
    
    // Visualize twist command
    subTwistCommand1 = node_->create_subscription<nav_msgs::msg::Path>(
        "/move_base/TrajectoryPlannerROS/local_plan", 5,
        std::bind(&TMPlanner::twistCommandHandler, this, std::placeholders::_1));
    
    subTwistCommand2 = node_->create_subscription<nav_msgs::msg::Path>(
        "/move_base/DWAPlannerROS/local_plan", 5,
        std::bind(&TMPlanner::twistCommandHandler, this, std::placeholders::_1));
    
    // Publisher - publishes velocity commands
    pubTwistCommand = node_->create_publisher<geometry_msgs::msg::Twist>("/twist_command", 5);
}

void TMPlanner::activate() {
    RCLCPP_INFO(node_->get_logger(), "TMPlanner activated");
}

void TMPlanner::deactivate() {
    RCLCPP_INFO(node_->get_logger(), "TMPlanner deactivated");
}

void TMPlanner::cleanup() {
    RCLCPP_INFO(node_->get_logger(), "TMPlanner cleanup");
}

// Visualize twist command - computes and publishes velocity commands
void TMPlanner::twistCommandHandler(const nav_msgs::msg::Path::SharedPtr pathMsg) {
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
        
        // If local plan is empty, do nothing
        if (pathMsg->poses.empty()) {
            return;
        }
        
        // Extract the first pose from the path
        geometry_msgs::msg::PoseStamped targetPose = pathMsg->poses[0];
        
        // Get current robot position and orientation
        double currentX = transform.transform.translation.x;
        double currentY = transform.transform.translation.y;
        
        // Compute the desired linear and angular velocities
        double dx = targetPose.pose.position.x - currentX;
        double dy = targetPose.pose.position.y - currentY;
        
        // Get current yaw from transform
        double roll, pitch, currentYaw;
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, currentYaw);
        
        double targetYaw = atan2(dy, dx);
        double yawError = targetYaw - currentYaw;
        
        // Normalize yaw error to the range [-pi, pi]
        while (yawError > M_PI) yawError -= 2 * M_PI;
        while (yawError < -M_PI) yawError += 2 * M_PI;
        
        // Set the desired speed (m/s)
        double desiredSpeed = 0.5; // Adjust this value as needed
        
        // Calculate the linear velocity components
        double linearVelocityX = desiredSpeed * cos(targetYaw);
        double linearVelocityY = desiredSpeed * sin(targetYaw);
        
        // Create and publish Twist command
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linearVelocityX;
        twist.linear.y = linearVelocityY;
        twist.angular.z = yawError;
        
        pubTwistCommand->publish(twist);
        
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "TF Error in twistCommandHandler: %s", ex.what());
        return;
    }
}

// Receive path from PRM global planner
void TMPlanner::pathHandler(const nav_msgs::msg::Path::SharedPtr pathMsg) {
    // If the planner couldn't find a feasible path, pose size should be 0
    globalPath = *pathMsg;
}

nav_msgs::msg::Path TMPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker)
{
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    plan.header.stamp = node_->now();

    // 1. Publish Goal to PRM Planner
    RCLCPP_INFO(node_->get_logger(), "Goal Received at: [%.2f, %.2f, %.2f]", 
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    pubGoal->publish(goal);

    // 2. If the planner couldn't find a feasible path, pose size should be 0
    if (globalPath.poses.empty()) {
        RCLCPP_INFO(node_->get_logger(), "No valid path found");
        return plan; // Return empty path
    }
    RCLCPP_INFO(node_->get_logger(), "A Valid Path Received!");

    // 3. Extract Path
    for (const auto & pose : globalPath.poses) {
        plan.poses.push_back(pose);
    }

    // Set the goal orientation for the last pose
    if (!plan.poses.empty()) {
        plan.poses.back().pose.orientation = goal.pose.orientation;
    }

    // Clear the stored path
    globalPath.poses.clear();

    return plan;
}

} // namespace tm_planner
