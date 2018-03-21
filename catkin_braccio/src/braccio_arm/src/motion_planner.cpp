#include <string>
#include <sstream>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometric_shapes/shape_operations.h>

#include <eigen_conversions/eigen_msg.h>

class TrajectorySampler
{
public:
  explicit TrajectorySampler(ros::NodeHandle nh);
  ~TrajectorySampler();

private:
  ros::NodeHandle nh_;

  int cycle_counter;
  std::string target_description_param;

  const std::string PLANNING_GROUP = "arm";
  const std::string GRIPPER_GROUP = "gripper";
  const std::string SHELF_MESH_PATH =
    "package://rosbot_v1/models/kinematics_shelf/kinematics_shelf.dae";
  const std::string BIN_MESH_PATH =
    "package://rosbot_v1/models/kinematics_bin/kinematics_bin.dae";


  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::planning_interface::MoveGroupInterface eef_group;

  const robot_state::JointModelGroup *joint_model_group;
  const robot_state::JointModelGroup *gripper_joint_model_group;

  // Define PlanningSceneInterface object to add and remove collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /*
   * Functions for gripper actuation
   * close_gripper = 0; open gripper
   *                 = 1; close gripper
   */
  bool OperateGripper(const bool &close_gripper);
  bool OpenGripper();
  bool CloseGripper();

  bool SetupCollisionObject(const std::string &object_id,
                            const std::string &mesh_path,
                            const geometry_msgs::Pose &object_pose,
                            moveit_msgs::CollisionObject &collision_object);
};


TrajectorySampler::TrajectorySampler(ros::NodeHandle nh)
  : nh_(nh),
    cycle_counter(0),
    move_group(PLANNING_GROUP),
    eef_group(GRIPPER_GROUP)
{
  /*
   * Setup:
   * Load robot model, robot state, set planning_scene
   * Define the move_group for planning and control purpose
   */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  robot_state::RobotState robot_kinematic_state(kinematic_model);

  // Get demo param
  bool demo;
  ros::param::get("trajectory_sampler/demo", demo);

  // set RRT as the planner and set allowed planning time
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(10.0);
  eef_group.setPlannerId("RRTConnectkConfigDefault");
  eef_group.setPlanningTime(50.0);

  // Pointer to JointModelGroup for improved performance.
  joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  gripper_joint_model_group =
    eef_group.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);


  while (ros::ok())
  {
    /*
     * rviz visualization:
     * Setup MoveItVisualTools for visualizing collision objects, robot,
     * and trajectories in Rviz
     */
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    // Load RemoteControl for step-by-step progression
    visual_tools.loadRemoteControl();

    // Create text marker for displaying current state
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    Eigen::Affine3d instr_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 4.0;
    instr_pose.translation().z() = 3.5;
    visual_tools.publishText(text_pose, "Welcome to pick-place project!",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    // visual_tools.publishText(instr_pose, "Press 'Next' to continue...",
    // rviz_visual_tools::GREEN, rviz_visual_tools::XXXLARGE);

    // Publish messages to rviz
    visual_tools.trigger();
    //visual_tools.prompt("next step");


    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = -0.2;
    target_pose.position.y = 0;
    target_pose.position.z = 0.2;


    // set starting pose
    move_group.setStartStateToCurrentState();

    // set target pose
    move_group.setPoseTarget(target_pose);

    // slow down movement of the robot
    move_group.setMaxVelocityScalingFactor(0.2);
    eef_group.setMaxVelocityScalingFactor(1.0);

    // define plan object which will hold the planned trajectory
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (bool)move_group.plan(my_plan);
    ROS_INFO("Visualizing plan to target: %s",
             success ? "SUCCEEDED" : "FAILED");

    // Visualize the plan
    visual_tools.publishAxisLabeled(target_pose, "target_pose");
    visual_tools.publishText(text_pose, "Displaying plan to target location",
                             rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    //visual_tools.prompt("next step");

    /*
     * Convert plan to a set of eef poses for IK calculation
     */
    robot_trajectory::RobotTrajectoryPtr robot_trajectory(
      new robot_trajectory::RobotTrajectory(kinematic_model,
                                            joint_model_group->getName()));

    // RobotState contains the current position/velocity/acceleration data
    moveit::core::RobotStatePtr robot_current_state;
    std::vector<double> robot_joint_positions;

    // Vector to hold eef poses
    std::vector<geometry_msgs::Pose> path;
    std::size_t path_size;

    // Declare service client for IK
/*    ros::ServiceClient client =
      nh.serviceClient<rosbot_v1::CalculateIK>("calculate_ik");
    rosbot_v1::CalculateIK srv;*/

    /*
     * Execute the plan based on demo flag
     * if demo = 1; use moveit for IK
     *         = 0; use IK_server for IK
     */
      // Display current state
      visual_tools.publishText(text_pose, "Moving to the target location",
                               rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE);
      visual_tools.trigger();
      // command the robot to execute the created plan
      success = (bool)move_group.execute(my_plan);
      ROS_INFO("Moving to pick location: %s",
               success ? "SUCCEEDED" : "FAILED");


  }
}

bool TrajectorySampler::OperateGripper(const bool &close_gripper)
{
  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state =
    eef_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group,
      gripper_joint_positions);

  ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper)
  {
    gripper_joint_positions[0] = 0.02;  // radians
    gripper_joint_positions[1] = 0.02;  // radians
  }
  else
  {
    gripper_joint_positions[0] = -0.01;  // radians
    gripper_joint_positions[1] = -0.01;  // radians
  }

  eef_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.5).sleep();

  bool success = (bool)eef_group.move();
  return success;
}

bool TrajectorySampler::OpenGripper()
{
  bool success = OperateGripper(false);
  ROS_INFO("Gripper actuation: Opening %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}

bool TrajectorySampler::CloseGripper()
{
  bool success = OperateGripper(true);
  ROS_INFO("Gripper actuation: Closing %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}

bool TrajectorySampler::SetupCollisionObject(const std::string &object_id,
    const std::string &mesh_path,
    const geometry_msgs::Pose &object_pose,
    moveit_msgs::CollisionObject &collision_object)
{
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = object_id;

  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);

  ROS_DEBUG_STREAM(object_id << " mesh loaded");

  shape_msgs::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m, object_mesh_msg);
  object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = object_mesh;

  collision_object.mesh_poses[0].position = object_pose.position;
  collision_object.mesh_poses[0].orientation = object_pose.orientation;

  collision_object.meshes.push_back(object_mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;
}


TrajectorySampler::~TrajectorySampler() {}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_sampler");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  TrajectorySampler plan_sampler(nh);
  return 0;
}
