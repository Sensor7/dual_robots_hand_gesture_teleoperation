#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/solid_primitive_dims.h>
namespace rvt = rviz_visual_tools;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("logger_my_moveit_cpp");


void constructGoalConstraints(
  moveit_msgs::msg::Constraints& goal,
  const int& index, 
  const std::string& link_name,
  const geometry_msgs::msg::PoseStamped& pose,
  const std::vector<double>& tolerance_pos,
  const std::vector<double>& tolerance_angle)
{
  // moveit_msgs::msg::Constraints goal;

  // goal.position_constraints.resize(1);
  moveit_msgs::msg::PositionConstraint& pcm = goal.position_constraints[index];
  pcm.link_name = link_name;
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  shape_msgs::msg::SolidPrimitive& bv = pcm.constraint_region.primitives[0];
  if (tolerance_pos.size()<3)
  {
    bv.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    bv.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::SPHERE>());
    bv.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = tolerance_pos[0];
  }
  else
  {
    bv.type = shape_msgs::msg::SolidPrimitive::BOX;
    bv.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>());
    bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = tolerance_pos[0];
    bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = tolerance_pos[1];
    bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = tolerance_pos[2];
  }
  
  pcm.header = pose.header;
  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position = pose.pose.position;

  // orientation of constraint region does not affect anything, since it is a sphere
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  // goal.orientation_constraints.resize(1);
  moveit_msgs::msg::OrientationConstraint& ocm = goal.orientation_constraints[index];
  ocm.link_name = link_name;
  ocm.header = pose.header;
  ocm.orientation = pose.pose.orientation;
  if (tolerance_angle.size() < 3)
  {
    ocm.absolute_x_axis_tolerance = tolerance_angle[0];
    ocm.absolute_y_axis_tolerance = tolerance_angle[0];
    ocm.absolute_z_axis_tolerance = tolerance_angle[0];
  }
  else
  {
    ocm.absolute_x_axis_tolerance = tolerance_angle[0];
    ocm.absolute_y_axis_tolerance = tolerance_angle[1];
    ocm.absolute_z_axis_tolerance = tolerance_angle[2];
  }
  ocm.weight = 1.0;

}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");

  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("node_my_moveit_cpp", "", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  static const std::string PLANNING_GROUP = "dual_manipulator";

  /* Otherwise robot with zeros joint_states */
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "world", "dual_panda_demo",
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Planning with MoveItCpp
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // There are multiple ways to set the start and the goal states of the plan
  // they are illustrated in the following plan examples
  //
  // Plan #1
  // ^^^^^^^
  //
  // We can set the start state of the plan to the current state of the robot
  planning_components->setStartStateToCurrentState();

  // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
  geometry_msgs::msg::PoseStamped target_pose_L,target_pose_R;
  target_pose_L.header.frame_id = "world";
  target_pose_L.pose.orientation.w = 1.0;
  target_pose_L.pose.position.x = 0.28;
  target_pose_L.pose.position.y = -0.2+0.5;
  target_pose_L.pose.position.z = 0.5;

  target_pose_R.header.frame_id = "world";
  target_pose_R.pose.orientation.w = 1.0;
  target_pose_R.pose.position.x = 0.28;
  target_pose_R.pose.position.y = -0.2-0.5;
  target_pose_R.pose.position.z = 0.5;

  std::vector<double> tolerance_pos(3,0.001);
  std::vector<double> tolerance_angle(3,0.001);
  // moveit_msgs::msg::Constraints goal_constraint_L = 
  //   kinematic_constraints::constructGoalConstraints("L_panda_link8", target_pose_L, p_tolerances, r_tolerances);
  // moveit_msgs::msg::Constraints goal_constraint_R = 
  //   kinematic_constraints::constructGoalConstraints("R_panda_link8", target_pose_R, p_tolerances, r_tolerances);
  // planning_components->setGoal(target_pose_L, "L_panda_link8");
  // planning_components->setGoal(target_pose_R, "R_panda_link8");

  moveit_msgs::msg::Constraints goal;
  goal.position_constraints.resize(2);
  goal.orientation_constraints.resize(2);
  constructGoalConstraints(goal, 0, "L_panda_link8", target_pose_L, tolerance_pos, tolerance_angle);
  constructGoalConstraints(goal, 1, "R_panda_link8", target_pose_R, tolerance_pos, tolerance_angle);

  planning_components->setGoal({goal});


  // Now, we call the PlanningComponents to compute the plan and visualize it.
  // Note that we are just planning
  auto plan_solution1 = planning_components->plan();

  RCLCPP_INFO(LOGGER, "Visualizing %s", plan_solution1 ? "" : "FAILED");


  // Check if PlanningComponents succeeded in finding the plan
  if (plan_solution1)
  {
    // Visualize the start pose in Rviz
    visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("L_panda_link8"), "start_pose_L");
    visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("R_panda_link8"), "start_pose_R");
    // Visualize the goal pose in Rviz
    visual_tools.publishAxisLabeled(target_pose_L.pose, "target_pose_L");
    visual_tools.publishAxisLabeled(target_pose_R.pose, "target_pose_R");
    visual_tools.publishText(text_pose, "setStartStateToCurrentState", rvt::WHITE, rvt::XLARGE);
    // Visualize the trajectory in Rviz
    visual_tools.publishTrajectoryLine(plan_solution1.trajectory, moveit_cpp_ptr->getRobotModel()->getLinkModel("L_panda_link8"));
    visual_tools.publishTrajectoryLine(plan_solution1.trajectory, moveit_cpp_ptr->getRobotModel()->getLinkModel("R_panda_link8"));
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    /* planning_components->execute(); // Execute the plan */
  }

  // // Plan #1 visualization:
  // //
  // // .. image:: images/moveitcpp_plan1.png
  // //    :width: 250pt
  // //    :align: center
  // //
  // // Start the next plan
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // visual_tools.deleteAllMarkers();
  // visual_tools.trigger();

  // // Plan #2
  // // ^^^^^^^
  // //
  // // Here we will set the current state of the plan using
  // // moveit::core::RobotState
  // auto start_state = *(moveit_cpp_ptr->getCurrentState());
  // geometry_msgs::msg::Pose start_pose;
  // start_pose.orientation.w = 1.0;
  // start_pose.position.x = 0.55;
  // start_pose.position.y = 0.0;
  // start_pose.position.z = 0.6;

  // start_state.setFromIK(joint_model_group_ptr, start_pose);

  // planning_components->setStartState(start_state);

  // // We will reuse the old goal that we had and plan to it.
  // auto plan_solution2 = planning_components->plan();
  // if (plan_solution2)
  // {
  //   moveit::core::RobotState robot_state(robot_model_ptr);
  //   moveit::core::robotStateMsgToRobotState(plan_solution2.start_state, robot_state);

  //   visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("panda_link8"), "start_pose");
  //   visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
  //   visual_tools.publishText(text_pose, "moveit::core::RobotState_Start_State", rvt::WHITE, rvt::XLARGE);
  //   visual_tools.publishTrajectoryLine(plan_solution2.trajectory, joint_model_group_ptr);
  //   visual_tools.trigger();

  //   /* Uncomment if you want to execute the plan */
  //   /* planning_components->execute(); // Execute the plan */
  // }

  // // Plan #2 visualization:
  // //
  // // .. image:: images/moveitcpp_plan2.png
  // //    :width: 250pt
  // //    :align: center
  // //
  // // Start the next plan
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // visual_tools.deleteAllMarkers();
  // visual_tools.trigger();

  // // Plan #3
  // // ^^^^^^^
  // //
  // // We can also set the goal of the plan using
  // // moveit::core::RobotState
  // auto target_state = *robot_start_state;
  // geometry_msgs::msg::Pose target_pose2;
  // target_pose2.orientation.w = 1.0;
  // target_pose2.position.x = 0.55;
  // target_pose2.position.y = -0.05;
  // target_pose2.position.z = 0.8;

  // target_state.setFromIK(joint_model_group_ptr, target_pose2);

  // planning_components->setGoal(target_state);

  // // We will reuse the old start that we had and plan from it.
  // auto plan_solution3 = planning_components->plan();
  // if (plan_solution3)
  // {
  //   moveit::core::RobotState robot_state(robot_model_ptr);
  //   moveit::core::robotStateMsgToRobotState(plan_solution3.start_state, robot_state);

  //   visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("panda_link8"), "start_pose");
  //   visual_tools.publishAxisLabeled(target_pose2, "target_pose");
  //   visual_tools.publishText(text_pose, "moveit::core::RobotState_Goal_Pose", rvt::WHITE, rvt::XLARGE);
  //   visual_tools.publishTrajectoryLine(plan_solution3.trajectory, joint_model_group_ptr);
  //   visual_tools.trigger();

  //   /* Uncomment if you want to execute the plan */
  //   /* planning_components->execute(); // Execute the plan */
  // }

  // // Plan #3 visualization:
  // //
  // // .. image:: images/moveitcpp_plan3.png
  // //    :width: 250pt
  // //    :align: center
  // //
  // // Start the next plan
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // visual_tools.deleteAllMarkers();
  // visual_tools.trigger();

  // // Plan #4
  // // ^^^^^^^
  // //
  // // We can set the start state of the plan to the current state of the robot
  // // We can set the goal of the plan using the name of a group states
  // // for panda robot we have one named robot state for "panda_arm" planning group called "ready"
  // // see `panda_arm.xacro
  // // <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/config/panda_arm.xacro#L13>`_

  // /* // Set the start state of the plan from a named robot state */
  // /* planning_components->setStartState("ready"); // Not implemented yet */
  // // Set the goal state of the plan from a named robot state
  // planning_components->setGoal("ready");

  // // Again we will reuse the old start that we had and plan from it.
  // auto plan_solution4 = planning_components->plan();
  // if (plan_solution4)
  // {
  //   moveit::core::RobotState robot_state(robot_model_ptr);
  //   moveit::core::robotStateMsgToRobotState(plan_solution4.start_state, robot_state);

  //   visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("panda_link8"), "start_pose");
  //   visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("panda_link8"), "target_pose");
  //   visual_tools.publishText(text_pose, "Goal_Pose_From_Named_State", rvt::WHITE, rvt::XLARGE);
  //   visual_tools.publishTrajectoryLine(plan_solution4.trajectory, joint_model_group_ptr);
  //   visual_tools.trigger();

  //   /* Uncomment if you want to execute the plan */
  //   /* planning_components->execute(); // Execute the plan */
  // }

  // // Plan #4 visualization:
  // //
  // // .. image:: images/moveitcpp_plan4.png
  // //    :width: 250pt
  // //    :align: center
  // //
  // // Start the next plan
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // visual_tools.deleteAllMarkers();
  // visual_tools.trigger();

  // // Plan #5
  // // ^^^^^^^
  // //
  // // We can also generate motion plans around objects in the collision scene.
  // //
  // // First we create the collision object
  // moveit_msgs::msg::CollisionObject collision_object;
  // collision_object.header.frame_id = "panda_link0";
  // collision_object.id = "box";

  // shape_msgs::msg::SolidPrimitive box;
  // box.type = box.BOX;
  // box.dimensions = { 0.1, 0.4, 0.1 };

  // geometry_msgs::msg::Pose box_pose;
  // box_pose.position.x = 0.4;
  // box_pose.position.y = 0.0;
  // box_pose.position.z = 1.0;

  // collision_object.primitives.push_back(box);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // // Add object to planning scene
  // {  // Lock PlanningScene
  //   planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
  //   scene->processCollisionObjectMsg(collision_object);
  // }  // Unlock PlanningScene
  // planning_components->setStartStateToCurrentState();
  // planning_components->setGoal("extended");

  // auto plan_solution5 = planning_components->plan();
  // if (plan_solution5)
  // {
  //   visual_tools.publishText(text_pose, "Planning_Around_Collision_Object", rvt::WHITE, rvt::XLARGE);
  //   visual_tools.publishTrajectoryLine(plan_solution5.trajectory, joint_model_group_ptr);
  //   visual_tools.trigger();

  //   /* Uncomment if you want to execute the plan */
  //   /* planning_components->execute(); // Execute the plan */
  // }

  // // Plan #5 visualization:
  // //
  // // .. image:: images/moveitcpp_plan5.png
  // //    :width: 250pt
  // //    :align: center
  // //
  // // END_TUTORIAL
  // visual_tools.prompt("Press 'next' to end the demo");
  // visual_tools.deleteAllMarkers();
  // visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}