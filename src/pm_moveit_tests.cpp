#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  //auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());
  });

  rclcpp::Parameter simTime( "use_sim_time", rclcpp::ParameterValue(true));
  node->set_parameter( simTime );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  //auto move_group_interface = MoveGroupInterface(node, "PM_Robot_Laser_TCP");
  //auto move_group_interface = MoveGroupInterface(node, "PM_Robot_Tool_TCP");
  auto move_group_interface = MoveGroupInterface(node, "PM_Robot_Cam1_TCP");
  //robot_model_loader::RobotModelLoader robot_model_loader(node);
  //const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  //RCLCPP_INFO(logger, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  //moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  //robot_state->setToDefaultValues();
  //const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("PM_Robot_Tool_TCP");
  //const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()-> getJointModelGroup("PM_Robot_Tool_TCP");
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()-> getJointModelGroup("PM_Robot_Cam1_TCP");

  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
   // We can retrieve the current set of joint values stored in the state for the Panda arm.
  std::vector<double> joint_values;
  current_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_WARN(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  RCLCPP_INFO(logger, "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(), move_group_interface.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();

  
  RCLCPP_WARN(logger, "Endeffector Pose: %f", current_pose.pose.position.x);
  RCLCPP_WARN(logger, "Endeffector Pose: %f", current_pose.pose.position.y);
  RCLCPP_WARN(logger, "Endeffector Pose: %f", current_pose.pose.position.z);
  RCLCPP_WARN(logger, "Endeffector Orientation: %f", current_pose.pose.orientation.w);

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    //msg.orientation.w = 1.0;
    //msg.position.x = 0.62;
    //msg.position.y = 0.39935;
    //msg.position.z = 1.3544;
    msg.orientation.w = 1.0;
    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = 0;
    return msg;
  }();
  current_pose.pose.position.x += 0.001;

  RCLCPP_WARN(logger, "Changed State");
  RCLCPP_WARN(logger, "Endeffector Pose: %f", current_pose.pose.position.x);
  RCLCPP_WARN(logger, "Endeffector Pose: %f", current_pose.pose.position.y);
  RCLCPP_WARN(logger, "Endeffector Pose: %f", current_pose.pose.position.z);
  RCLCPP_WARN(logger, "Endeffector Orientation: %f", current_pose.pose.orientation.w);

  // moveit::core::RobotStatePtr target_state = move_group_interface.getCurrentState(10);
  // std::vector<double> target_joint_values = move_group_interface.getJointValueTarget();
  // target_state->copyJointGroupPositions(joint_model_group, target_joint_values);

  geometry_msgs::msg::PoseStamped rand_pose = move_group_interface.getRandomPose();
    //double x = pose.position.x; 
    //double y = pose.position.y;
    //double z = pose.position.z;

  RCLCPP_WARN(logger, "Random Endeffector Pose");
  RCLCPP_WARN(logger, "Rand Endeffector Pose: %f", rand_pose.pose.position.x);
  RCLCPP_WARN(logger, "Rand Endeffector Pose: %f", rand_pose.pose.position.y);
  RCLCPP_WARN(logger, "Rand Endeffector Pose: %f", rand_pose.pose.position.z);
  RCLCPP_WARN(logger, "Rand Endeffector Orientation: %f", rand_pose.pose.orientation.w);

  //setGoalPositionTolerance and setGoalOrientationTolerance
  
  //move_group_interface.setPoseTarget(target_pose);
  
  std::vector<double> joint_group_positions;
  RCLCPP_INFO(logger, "state received!");

  //current_state->copyJointGroupPositions(joint_model_group,joint_group_positions);
  move_group_interface.setPlanningTime(10);

  //robot_state->setJointGroupPositions(joint_model_group, joint_values);


  /* Check whether any joint is outside its joint limits */
  RCLCPP_INFO_STREAM(logger, "Current state is " << (current_state->satisfiesBounds() ? "valid" : "not valid"));
  move_group_interface.setPoseTarget(current_pose);
  //move_group_interface.setPoseTarget(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, "Cam1_Toolhead_TCP");
  move_group_interface.setGoalTolerance(0.5);
  //move_group_interface.setGoalPositionTolerance(0.1);
  //joint_values[0]=joint_values[0]+0.001;
  //move_group_interface.setJointValueTarget(joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(logger, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  
  // // Create a plan to that target pose
  // auto const [success, plan] = [&move_group_interface]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}