#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

bool infinite()
{
	while(1) {RCLCPP_INFO(LOGGER, "WTF!!!");}
	return true;
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
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

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
  static const std::string PLANNING_GROUP = "robot";
  static const std::string LOGNAME = "moveit_cpp_tutorial";

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
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "moveit_cpp_tutorial",
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

	planning_components->setStartStateToCurrentState();
  
  
   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  
  // Get current joint value
  
 	planning_scene::PlanningScene planning_scene(robot_model_ptr);
  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  
 	std::vector<double> joint_values;
	const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("robot");
	planning_components->getStartState()->copyJointGroupPositions(joint_model_group, joint_values);
	
	
	
	RCLCPP_INFO(LOGGER, "**************  JOINT VALUES   *********************");
	for (int i=0; i<(int)joint_values.size(); i++)
	{
		RCLCPP_INFO(LOGGER, "joint n°%i : %f", i, joint_values[i]);
	}
	
  
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  
  
  // Checking collision
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  
 	
  collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	planning_scene.checkCollision(collision_request, collision_result);
	RCLCPP_INFO_STREAM(LOGGER, "Test 1: Current state is " << (collision_result.collision ? "in" : "not in")
                                                       << " collision");
  collision_request.contacts = true;
	collision_request.max_contacts = 1000;

	collision_result.clear();
	planning_scene.checkSelfCollision(collision_request, collision_result);
	RCLCPP_INFO_STREAM(LOGGER, "Test 2: Current state is " << (collision_result.collision ? "in" : "not in")
		                                                     << " collision");
	collision_detection::CollisionResult::ContactMap::const_iterator it;
	for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
	{
		RCLCPP_INFO(LOGGER, "Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
	}
                                        
                                        

  

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  // Plan #5
  // ^^^^^^^
  //
  // We can also generate motion plans around objects in the collision scene.
  //
  // First we create the collision object
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.1, 0.4, 0.1 };

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = -0.2;
  box_pose.position.y = 0.0;
  box_pose.position.z = 1.1;
  //box_pose.position.x = 0.0;
  //box_pose.position.y = 0.0;
  //box_pose.position.z = 0.20;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Add object to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(collision_object);
  }  // Unlock PlanningScene
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
       moveit_cpp_ptr->getPlanningSceneMonitorNonConst();
       
	planning_scene_monitor->updateFrameTransforms();
	planning_scene_monitor.reset(); // release this pointer
  
  
  // Checking collision
  
	collision_result.clear();
	{  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
    scene->checkCollision(collision_request, collision_result);
    RCLCPP_INFO_STREAM(LOGGER, "Test 3: Current state is " << (collision_result.collision ? "in" : "not in")
                                                       << " collision");
  }  // Unlock PlanningScene
  
   
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  
 
  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  
  while (1)
  {
		 visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move your Rover");             
		                     
		
		moveit::core::RobotState& futur_state = planning_scene.getCurrentStateNonConst();
		joint_values[3] = joint_values[3] + 0.2;
		futur_state.setJointGroupPositions(joint_model_group, joint_values);
		
		collision_result.clear();		
		{  // Lock PlanningScene
		  planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
		  scene->checkCollision(collision_request, collision_result, futur_state);
		}  // Unlock PlanningScene
			
		if (!collision_result.collision)
		{
			RCLCPP_INFO_STREAM(LOGGER,"Futur state considered as valid");
			planning_components->setGoal(futur_state);
			auto plan_solution = planning_components->plan();
			
			if (plan_solution)  planning_components->execute();
			
		}
		
		else
		{
			
			RCLCPP_INFO_STREAM(LOGGER, "Warning: obstacle in front of Rover, please change your course !");
		
		}
		
		
  }
  
 


	
  // Plan #5 visualization:
  //
  // .. image:: images/moveitcpp_plan5.png
  //    :width: 250pt
  //    :align: center
  //
  // END_TUTORIAL
  visual_tools.prompt("Press 'next' to end the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}
