/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
  Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <rclcpp/node.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory> // For std::shared_ptr

// MoveIt
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_task_constructor_demo/pick_place_demo_parameters.hpp>
#include <grasp_msgs/msg/grasp_config_list.hpp>

#pragma once

namespace moveit_task_constructor_demo {
using namespace moveit::task_constructor;

// prepare a demo environment from ROS parameters under node
void setupDemoScene(const pick_place_task_demo::Params& params);

class PickPlaceTask
{
public:
	PickPlaceTask(const std::string& task_name);
	~PickPlaceTask() = default;

	/**
	 * @brief Initialize the task
	 * @param node The ROS node to use for parameters and planning scene
	 * @param params Task parameters
	 * @param grasp_list Shared pointer to the received grasp list message
	 * @return True if initialization is successful, false otherwise
	 */
	bool init(const rclcpp::Node::SharedPtr& node,
	          const pick_place_task_demo::Params& params,
	          grasp_msgs::msg::GraspConfigList::ConstSharedPtr grasp_list);

	bool plan(const std::size_t max_solutions);

	bool execute();

private:
	std::string task_name_;
	moveit::task_constructor::TaskPtr task_;
	pick_place_task_demo::Params params_;
	moveit::planning_interface::PlanningSceneInterfacePtr psi_;
	rclcpp::Node::SharedPtr node_;
	grasp_msgs::msg::GraspConfigList::ConstSharedPtr grasp_list_;

	// TF Buffer and Listener
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	moveit::task_constructor::solvers::PipelinePlannerPtr
	createPlanner(const std::string& name = "ompl", const std::string& planning_pipeline = "ompl");
};
}  // namespace moveit_task_constructor_demo
