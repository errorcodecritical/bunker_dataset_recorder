#include "utility.h"
// this is a plugin that receives path from PRM global planner and pass it to nav2
#ifndef TM_PLANNER_H
#define TM_PLANNER_H

#include <functional>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_core/global_planner.hpp>

namespace tm_planner
{

class TMPlanner : public nav2_core::GlobalPlanner
{
public:
    TMPlanner() = default;
    ~TMPlanner() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void activate() override;
    void deactivate() override;
    void cleanup() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancel_checker) override;

private:
    // Handler functions
    void pathHandler(const nav_msgs::msg::Path::SharedPtr pathMsg);
    void twistCommandHandler(const nav_msgs::msg::Path::SharedPtr pathMsg);
    // ROS2 members
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    
    // Subscriptions and publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubGoal;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subTwistCommand1;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subTwistCommand2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubTwistCommand;
    
    // Stored path from PRM planner
    nav_msgs::msg::Path globalPath;
};

}  // namespace tm_planner

#endif
