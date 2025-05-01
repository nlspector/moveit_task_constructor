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
#include <rclcpp/rclcpp.hpp>
#include <memory> // For std::make_shared
#include <atomic> // For std::atomic
#include <chrono> // For std::chrono::seconds
#include <grasp_msgs/msg/grasp_config_list.hpp> // <-- ADD THIS INCLUDE

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/pick_place_task.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_demo");

// --- Global variables for subscriber ---
std::atomic<bool> g_grasps_received = false;
grasp_msgs::msg::GraspConfigList::ConstSharedPtr g_latest_grasps;
rclcpp::Subscription<grasp_msgs::msg::GraspConfigList>::SharedPtr g_grasp_subscriber; // Keep subscriber alive

// --- Subscriber Callback ---
void graspCallback(const grasp_msgs::msg::GraspConfigList::ConstSharedPtr msg)
{
	RCLCPP_INFO(LOGGER, "Received grasp list with %zu grasps.", msg->grasps.size());
	g_latest_grasps = msg;
	g_grasps_received = true;
	// Optionally shut down subscriber after first message if you only need one list
	// g_grasp_subscriber.reset();
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto node = rclcpp::Node::make_shared("moveit_task_constructor_demo", node_options);
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	const auto param_listener = std::make_shared<pick_place_task_demo::ParamListener>(node);
	const auto params = param_listener->get_params();
	moveit_task_constructor_demo::setupDemoScene(params);

	// --- Create subscriber ---
	const std::string grasp_topic = "/grasp_library/clustered_grasps"; // Use your topic name
	RCLCPP_INFO(LOGGER, "Subscribing to grasp topic: %s", grasp_topic.c_str());
	g_grasp_subscriber = node->create_subscription<grasp_msgs::msg::GraspConfigList>(
	    grasp_topic, 10, // QoS depth
	    graspCallback);

	// --- Wait for grasps ---
	RCLCPP_INFO(LOGGER, "Waiting for grasp list on topic %s...", grasp_topic.c_str());
	while (!g_grasps_received && rclcpp::ok())
	{
		// Spin briefly to allow callbacks to process
		// rclcpp::spin_some(node);
		// Instead of spin_some, just sleep, as executor is running in separate thread
		rclcpp::sleep_for(std::chrono::milliseconds(100));
	}

	if (!rclcpp::ok())
	{
		RCLCPP_ERROR(LOGGER, "ROS shutdown while waiting for grasps.");
		spinning_thread.join();
		return 1;
	}

	if (!g_latest_grasps || g_latest_grasps->grasps.empty())
	{
		RCLCPP_ERROR(LOGGER, "Received empty or null grasp list. Cannot plan.");
		spinning_thread.join();
		return 1;
	}

	// Construct and initialize pick/place task AFTER receiving grasps
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task");
	if (!pick_place_task.init(node, params, g_latest_grasps)) { // <-- PASS GRASP LIST
		RCLCPP_ERROR(LOGGER, "Initialization failed");
		spinning_thread.join();
		return 1;
	}

	if (pick_place_task.plan(params.max_solutions)) {
		RCLCPP_INFO(LOGGER, "Planning succeeded");
		if (params.execute) {
			pick_place_task.execute();
			RCLCPP_INFO(LOGGER, "Execution complete");
		} else {
			RCLCPP_INFO(LOGGER, "Execution disabled");
		}
	} else {
		RCLCPP_INFO(LOGGER, "Planning failed");
	}

	// Keep introspection alive
	spinning_thread.join();
	return 0;
}
