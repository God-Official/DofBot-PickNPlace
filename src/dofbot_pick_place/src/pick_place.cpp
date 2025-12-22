#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class DofbotTFPickPlace : public rclcpp::Node
{
public:
  DofbotTFPickPlace()
  : Node("dofbot_tf_pick_place"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(get_logger(), "🚀 DOFBOT TF Picker Started");

    // Delay MoveGroup init → avoids shared_from_this crash
    init_timer_ = create_wall_timer(
      500ms, std::bind(&DofbotTFPickPlace::initializeMoveIt, this));
  }

private:

  rclcpp::TimerBase::SharedPtr init_timer_;

  void initializeMoveIt()
  {
    init_timer_->cancel();

    RCLCPP_INFO(get_logger(), "🦾 Initializing MoveIt interfaces...");

    // NOW shared_from_this() is valid
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");

    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");

    arm_group_->setPlanningTime(5.0);
    arm_group_->setMaxVelocityScalingFactor(0.8);
    arm_group_->setMaxAccelerationScalingFactor(0.8);

    gripper_group_->setPlanningTime(3.0);
    gripper_group_->setMaxVelocityScalingFactor(1.0);
    gripper_group_->setMaxAccelerationScalingFactor(1.0);

    // Now safe to start sequence
    askColor();
  }

  // ------------------------------------------------------------
  // ASK USER FOR COLOR (MAIN LOOP)
  // ------------------------------------------------------------
  void askColor()
  {
    std::string color;
    std::cout << "\nEnter block color (red/green/blue/yellow/exit): ";
    std::cin >> color;

    if (color == "exit") {
      rclcpp::shutdown();
      return;
    }

    selected_tf_ = color + "_block_base";

    RCLCPP_INFO(get_logger(), "🎯 Selected TF: %s", selected_tf_.c_str());

    lookupTFAndStart();
  }

  // ------------------------------------------------------------
  // Lookup TF position
  // ------------------------------------------------------------
  void lookupTFAndStart()
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_.lookupTransform(
        "base_link", selected_tf_, tf2::TimePointZero, 2s);
    }
    catch (...) {
      RCLCPP_ERROR(get_logger(), "❌ TF '%s' NOT found. Try again.", selected_tf_.c_str());
      askColor();
      return;
    }

    pick_x_ = tf_msg.transform.translation.x;
    pick_y_ = tf_msg.transform.translation.y;
    pick_z_ = tf_msg.transform.translation.z;

    RCLCPP_INFO(get_logger(), "📍 TF: x=%.3f  y=%.3f  z=%.3f", pick_x_, pick_y_, pick_z_);

    startPickSequence();
  }

  // ------------------------------------------------------------
  // PICK → LIFT → HOME
  // ------------------------------------------------------------
  void startPickSequence()
  {
    moveNamed(arm_group_, "home");
    moveNamed(gripper_group_, "open");

    // Approach
    // if (!moveXYZ(pick_x_, pick_y_, pick_z_ + 0.10))
    //   return errorRestart("Approach failed");

    // Lower
    if (!moveXYZ(pick_x_, pick_y_, pick_z_ ))
      return errorRestart("pick failed");

    // Grip
    if (!moveNamed(gripper_group_, "close"))
      return errorRestart("Grip failed");

    // Lift
    if (!moveNamed(arm_group_, "stand"))
      return errorRestart("Lift failed");

    if (!moveXYZ( 0.2, 0.06, 0.05 ))
      return errorRestart("drop failed");
    
    if (!moveNamed(gripper_group_, "open"))
      return errorRestart("Grip_open failed");

    // Go home
    moveNamed(arm_group_, "home");
    moveNamed(gripper_group_, "open");

    RCLCPP_INFO(get_logger(), "🎉 Pick Completed!");

    askColor();   // loop again
  }

  void errorRestart(const std::string &msg)
  {
    RCLCPP_ERROR(get_logger(), "❌ %s", msg.c_str());
    moveNamed(arm_group_, "home");
    askColor();
  }

  // ------------------------------------------------------------
  // Move helpers
  // ------------------------------------------------------------
  bool moveNamed(
      const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group,
      const std::string &name)
  {
    group->setNamedTarget(name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      group->execute(plan);
      return true;
    }
    return false;
  }

  bool moveXYZ(double x, double y, double z)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;

    p.orientation.x = -0.5;
    p.orientation.y = -0.5;
    p.orientation.z = -0.5;
    p.orientation.w = 0.5;

    arm_group_->setPoseTarget(p);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      arm_group_->execute(plan);
      return true;
    }
    return false;
  }

  // ------------------------------------------------------------
  // Variables
  // ------------------------------------------------------------
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  std::string selected_tf_;
  double pick_x_, pick_y_, pick_z_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DofbotTFPickPlace>());
  rclcpp::shutdown();
  return 0;
}
