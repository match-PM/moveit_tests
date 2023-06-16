#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit3",
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


  moveit_msgs::msg::CollisionObject collision_object2;
  collision_object2.header.frame_id = "Gonio_Right_Chuck";
  collision_object2.id = "case";

  shapes::Mesh * m = shapes::createMeshFromResource("package://pm_robot_description/meshes/Gonio_Left/Gonio_Left_Chucks/Chuck_Siemens_Carrier_Gonio_Left.STL");
  shape_msgs::msg::Mesh shelf_mesh;
  shapes::ShapeMsg shelf_mesh_msg;
  bool success = shapes::constructMsgFromShape(m,shelf_mesh_msg);
  RCLCPP_INFO(logger, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::msg::Pose shelf_pose;

  shelf_pose.position.x =  0;
  shelf_pose.position.y =  0;
  shelf_pose.position.z =  0.1;
  collision_object2.meshes.push_back(shelf_mesh);
  collision_object2.mesh_poses.push_back(shelf_pose);
  //collision_object2.operation = collision_object2.ADD;


  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = "Gonio_Right_Chuck"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box111111111111111111111111111111111111111111111111111111111111111111111111111";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.01;
  primitive.dimensions[primitive.BOX_Y] = 0.01;
  primitive.dimensions[primitive.BOX_Z] = 0.01;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = 0.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
  }();
  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //planning_scene_interface.applyCollisionObject(collision_object);
  planning_scene_interface.applyCollisionObject(collision_object2);
  //move_group_interface.attachObject("case","Cam1_Toolhead_TCP");


  // Shutdown ROS
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}