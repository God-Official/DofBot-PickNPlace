#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class DofbotSmartMove : public rclcpp::Node
{
public:
  DofbotSmartMove()
  : Node("dofbot_smart_move"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    std::cout << "Enter block color (red/green/blue/yellow): ";
    std::cin >> selected_color_;

    selected_tf_ = selected_color_ + "_block_base";

    // ----------------------------
    // STACK PARAMETERS
    // ----------------------------
    cube_size_     = 0.03;   // 3 cm cube
    stack_offset_  = 0.002;  // 2 mm safety
    stack_count_   = 0;      // start empty stack

    // Base of stack (change if needed)
    stack_base_.x = 0.20;
    stack_base_.y = 0.0;
    stack_base_.z = 0.03;     // 3 cm above table (cube center)

    // ----------------------------
    // INIT TIMER
    // ----------------------------
    timer_init_ = this->create_wall_timer(
        1s,
        std::bind(&DofbotSmartMove::init, this));
  }

private:

  // ------------------------------------------------------------
  // INIT MOVEIT
  // ------------------------------------------------------------
  void init()
  {
    timer_init_->cancel();

    move_group_arm_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");

    move_group_gripper_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper");

    move_group_arm_->setPlanningTime(5.0);
    move_group_arm_->setMaxVelocityScalingFactor(0.4);

    // Move home
    move_group_arm_->setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (move_group_arm_->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        move_group_arm_->execute(home_plan);

    rclcpp::sleep_for(300ms);

    timer_tf_ = this->create_wall_timer(
        200ms,
        std::bind(&DofbotSmartMove::readTF, this));
  }

  // ------------------------------------------------------------
  // READ TF
  // ------------------------------------------------------------
  void readTF()
  {
    static int count = 0;
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_.lookupTransform("base_link", selected_tf_, tf2::TimePointZero);
    } catch (...) { return; }

    if (++count == 5)
    {
      timer_tf_->cancel();
      pick_position_.x = tf_msg.transform.translation.x;
      pick_position_.y = tf_msg.transform.translation.y;
      pick_position_.z = tf_msg.transform.translation.z;
      startPickSequence();
    }
  }

  // ------------------------------------------------------------
  // PICK → STACK SEQUENCE
  // ------------------------------------------------------------
  void startPickSequence()
  {
    goHome();
    openGripper();
    // ❌ NO detachBlock() HERE ANYMORE
    approachAboveBlock();
    lowerForCubeGrip();
    closeGripper();
    attachBlockToGripper();
    liftBlock();       // uses current pose now
    goHome();

    // ----------------------------
    // PERFECT STACKING
    // ----------------------------
    moveAboveStack();
    lowerToStack();
    openGripper();
    detachBlock();     // detach only when dropping
    rclcpp::sleep_for(200ms);

    // Increment stack level
    stack_count_++;

    liftToSafeZ();
    goHome();
  }

  // ------------------------------------------------------------
  // STACK POSITION COMPUTATION
  // ------------------------------------------------------------
  geometry_msgs::msg::Point getStackPosition()
  {
    geometry_msgs::msg::Point p = stack_base_;
    p.z = stack_base_.z + (stack_count_ * cube_size_);
    return p;
  }

  // ------------------------------------------------------------
  // MOVE ABOVE STACK AREA
  // ------------------------------------------------------------
  void moveAboveStack()
  {
    auto pos = getStackPosition();

    geometry_msgs::msg::Pose p;
    p.position.x = pos.x;
    p.position.y = pos.y;
    p.position.z = pos.z + 0.10; // 10 cm overhead

    setDownwardOrientation(p);
    move_group_arm_->setPoseTarget(p);
    executeArmPlan("move-above-stack");
  }

  // ------------------------------------------------------------
  // LOWER TO STACK EXACT HEIGHT
  // ------------------------------------------------------------
  void lowerToStack()
  {
    auto pos = getStackPosition();

    geometry_msgs::msg::Pose p;
    p.position.x = pos.x;
    p.position.y = pos.y;
    p.position.z = pos.z + (cube_size_ / 2.0) + stack_offset_;

    setDownwardOrientation(p);
    move_group_arm_->setPoseTarget(p);
    executeArmPlan("lower-to-stack");
  }

  // ------------------------------------------------------------
  // LIFT AFTER DROPPING
  // ------------------------------------------------------------
  void liftToSafeZ()
  {
    auto pos = getStackPosition();

    geometry_msgs::msg::Pose p;
    p.position.x = pos.x;
    p.position.y = pos.y;
    p.position.z = pos.z + 0.15;

    setDownwardOrientation(p);
    move_group_arm_->setPoseTarget(p);
    executeArmPlan("post-drop-lift");
  }

  // ------------------------------------------------------------
  // PICK BLOCK FUNCTIONS
  // ------------------------------------------------------------
  void approachAboveBlock()
  {
    geometry_msgs::msg::Pose p;
    p.position.x = pick_position_.x;
    p.position.y = pick_position_.y;
    p.position.z = pick_position_.z + 0.10;

    setDownwardOrientation(p);
    move_group_arm_->setPoseTarget(p);
    executeArmPlan("approach-above");
  }

  void lowerForCubeGrip()
  {
    double cube_half = 0.015;
    double clearance = 0.005;

    geometry_msgs::msg::Pose p;
    p.position.x = pick_position_.x;
    p.position.y = pick_position_.y;
    p.position.z = pick_position_.z - (cube_half - clearance);

    setDownwardOrientation(p);
    move_group_arm_->setPoseTarget(p);
    executeArmPlan("lower-for-grip");
  }

  // ------------------------------------------------------------
  // GRIPPER CONTROL
  // ------------------------------------------------------------
  void openGripper()
  {
    move_group_gripper_->setNamedTarget("open");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        move_group_gripper_->execute(plan);

    rclcpp::sleep_for(200ms);
  }

  void closeGripper()
  {
    move_group_gripper_->setNamedTarget("close");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        move_group_gripper_->execute(plan);

    rclcpp::sleep_for(500ms);
  }

  // ------------------------------------------------------------
  // ATTACH & DETACH BLOCK
  // ------------------------------------------------------------
  void attachBlockToGripper()
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = "picked_block";
    obj.header.frame_id = "Gripping_point_Link";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = { cube_size_, cube_size_, cube_size_ };

    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    p.position.x = 0.0;
    p.position.y = 0.0;
    p.position.z = -0.015;   // 1.5 cm below gripping point (closer to fingers)

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(p);
    obj.operation = obj.ADD;

    planning_scene_.applyCollisionObject(obj);
    move_group_arm_->attachObject("picked_block", "Gripping_point_Link");
  }

  void detachBlock()
  {
    move_group_arm_->detachObject("picked_block");
    planning_scene_.removeCollisionObjects({"picked_block"});
  }

  // ------------------------------------------------------------
  // LIFT BLOCK AFTER GRIPPING (FROM CURRENT POSE)
  // ------------------------------------------------------------
  void liftBlock()
  {
    // 🔥 IMPORTANT: use current pose, not pick_position_
    auto pose = move_group_arm_->getCurrentPose().pose;
    pose.position.z += 0.15;   // move straight up by 15 cm

    setDownwardOrientation(pose);
    move_group_arm_->setPoseTarget(pose);
    executeArmPlan("lift-block");
  }

  // ------------------------------------------------------------
  // HOME MOTION
  // ------------------------------------------------------------
  void goHome()
  {
    move_group_arm_->setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        move_group_arm_->execute(plan);

    rclcpp::sleep_for(300ms);
  }

  // ------------------------------------------------------------
  // ORIENTATION DOWNWARD
  // ------------------------------------------------------------
  void setDownwardOrientation(geometry_msgs::msg::Pose &p)
  {
    p.orientation.x = -0.5;
    p.orientation.y = -0.5;
    p.orientation.z = -0.5;
    p.orientation.w =  0.5;
  }

  // ------------------------------------------------------------
  // EXECUTE PLAN
  // ------------------------------------------------------------
  void executeArmPlan(const std::string &label)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_arm_->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
      move_group_arm_->execute(plan);
  }

  // ------------------------------------------------------------
  // VARIABLES
  // ------------------------------------------------------------
  std::string selected_color_;
  std::string selected_tf_;

  geometry_msgs::msg::Point pick_position_;

  // STACK VARIABLES
  geometry_msgs::msg::Point stack_base_;
  int stack_count_;
  double cube_size_;
  double stack_offset_;

  rclcpp::TimerBase::SharedPtr timer_init_;
  rclcpp::TimerBase::SharedPtr timer_tf_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DofbotSmartMove>());
  rclcpp::shutdown();
  return 0;
}