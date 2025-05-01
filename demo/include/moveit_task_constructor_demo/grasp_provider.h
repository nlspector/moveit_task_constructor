#ifndef MOVEIT_TASK_CONSTRUCTOR_DEMO_GRASP_PROVIDER_H
#define MOVEIT_TASK_CONSTRUCTOR_DEMO_GRASP_PROVIDER_H

#include <moveit/task_constructor/stage.h>
#include <grasp_msgs/msg/grasp_config_list.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <rclcpp/macros.hpp>
#include <tf2_ros/buffer.h>
#include <string>

namespace moveit_task_constructor_demo
{

/**
 * @brief A MoveIt Task Constructor stage that generates grasp poses based on a received GraspConfigList message.
 *        Inherits from Generator.
 */
class GraspProvider : public moveit::task_constructor::MonitoringGenerator
{
public:
	RCLCPP_SMART_PTR_DEFINITIONS(GraspProvider)

	GraspProvider(const std::string& name,
	              grasp_msgs::msg::GraspConfigList::ConstSharedPtr grasp_list,
	              planning_scene::PlanningSceneConstPtr initial_scene,
	              tf2_ros::Buffer::SharedPtr tf_buffer,
	              const std::string& target_frame);

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	// Generator requires compute() and canCompute()
	void compute() override;
	bool canCompute() const override;

	// Basic stage overrides (reset is virtual in base Stage)
	void reset() override;

protected:
	void onNewSolution(const moveit::task_constructor::SolutionBase& solution) override;
	ordered<const moveit::task_constructor::SolutionBase*> upstream_solutions_;
private:
	grasp_msgs::msg::GraspConfigList::ConstSharedPtr grasp_list_;
	planning_scene::PlanningSceneConstPtr initial_scene_;
	tf2_ros::Buffer::SharedPtr tf_buffer_;
	std::string target_frame_;
	bool generated_ = false;
};

} // namespace moveit_task_constructor_demo

#endif // MOVEIT_TASK_CONSTRUCTOR_DEMO_GRASP_PROVIDER_H 