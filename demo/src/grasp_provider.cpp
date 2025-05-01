#include <moveit_task_constructor_demo/grasp_provider.h>
#include <moveit/task_constructor/storage.h> // For InterfaceState
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/task.h> // Include Task header
#include <moveit/task_constructor/storage.h>
#include <grasp_msgs/msg/grasp_config.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For tf2::toMsg and transform
#include <tf2_ros/buffer.h> // For TF buffer
#include <tf2_ros/transform_listener.h> // Often used with buffer, though buffer might be enough
#include <rclcpp/logging.hpp>
#include <Eigen/Geometry> // For Isometry3d used in spawn()

namespace moveit_task_constructor_demo
{

namespace {
rclcpp::Logger getLogger() { return rclcpp::get_logger("GraspProvider"); }
} // namespace

GraspProvider::GraspProvider(const std::string& name,
                           grasp_msgs::msg::GraspConfigList::ConstSharedPtr grasp_list,
                           planning_scene::PlanningSceneConstPtr initial_scene,
                           tf2_ros::Buffer::SharedPtr tf_buffer,
                           const std::string& target_frame)
  : moveit::task_constructor::MonitoringGenerator(name)
  , grasp_list_(grasp_list)
  , initial_scene_(initial_scene)
  , tf_buffer_(tf_buffer)
  , target_frame_(target_frame)
{
	// Properties from GeneratePose base class are likely not needed/used here.
	if (!initial_scene_) {
		RCLCPP_WARN(getLogger(), "Initial planning scene provided to GraspProvider is null.");
	}
	if (!tf_buffer_) {
		RCLCPP_WARN(getLogger(), "TF buffer provided to GraspProvider is null.");
	}
	if (target_frame_.empty()) {
		RCLCPP_WARN(getLogger(), "Target frame provided to GraspProvider is empty.");
	}
}

void GraspProvider::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	// Call the base class init (Generator inherits from Stage)
	moveit::task_constructor::MonitoringGenerator::init(robot_model);
	if (!grasp_list_) {
		RCLCPP_WARN(getLogger(), "Grasp list provided to GraspProvider is null.");
	}
}

void GraspProvider::reset()
{
	// Call base reset
	upstream_solutions_.clear();
	moveit::task_constructor::MonitoringGenerator::reset();
	generated_ = false; // Reset the flag
}

bool GraspProvider::canCompute() const
{
	if (upstream_solutions_.empty())
		RCLCPP_WARN(getLogger(), "upstream solutions empty");
	// We can compute if we haven't generated yet AND have a valid grasp list
	return !generated_ && static_cast<bool>(grasp_list_) && !grasp_list_->grasps.empty() && !(upstream_solutions_.empty());
}

void GraspProvider::compute()
{
	// Double-check flag and grasp list (canCompute should prevent this state)
	if (generated_ || !grasp_list_) {
		return;
	}

	if (upstream_solutions_.empty())
		return;

	const moveit::task_constructor::SolutionBase& s = *upstream_solutions_.pop();
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene()->diff();

	// Use the stored initial planning scene
	// planning_scene::PlanningSceneConstPtr scene = initial_scene_;
	if (!scene) { // Check if the stored scene is valid
		RCLCPP_ERROR(getLogger(), "Stored initial planning scene is null.");
		return;
	}

	RCLCPP_INFO(getLogger(), "Generating %zu grasp poses from topic.", grasp_list_->grasps.size());

	for (const auto& grasp : grasp_list_->grasps) {
		geometry_msgs::msg::PoseStamped target_pose_msg;
		target_pose_msg.header = grasp_list_->header; // Use frame_id from the list header

		// --- Construct Pose from GraspConfig ---
		// Position: Use the 'surface' point
		target_pose_msg.pose.position = grasp.surface;

		// Orientation: Construct from approach, binormal, axis
		// Assuming convention: X=binormal, Y=axis, Z=-approach
		Eigen::Vector3d binormal(grasp.binormal.x, grasp.binormal.y, grasp.binormal.z);
		Eigen::Vector3d axis(grasp.axis.x, grasp.axis.y, grasp.axis.z);
		Eigen::Vector3d neg_approach(grasp.approach.x, grasp.approach.y, grasp.approach.z); // Invert approach

		// Normalize vectors
		binormal.normalize();
		axis.normalize();
		neg_approach.normalize();

		// Create rotation matrix
		tf2::Matrix3x3 rotation_matrix;
		rotation_matrix.setValue(neg_approach.x(), binormal.x(), axis.x(),
		                         neg_approach.y(), binormal.y(), axis.y(),
		                         neg_approach.z(), binormal.z(), axis.z());

		// Convert to Quaternion
		tf2::Quaternion quaternion;
		rotation_matrix.getRotation(quaternion);
		quaternion.normalize(); // Ensure it's a unit quaternion

		target_pose_msg.pose.orientation = tf2::toMsg(quaternion);
		// --- Pose Construction End ---

		// Target pose is now in the frame specified by grasp_list_->header.frame_id
		geometry_msgs::msg::PoseStamped final_pose_msg;
		if (tf_buffer_ && !target_frame_.empty() && target_pose_msg.header.frame_id != target_frame_) {
			try {
				// Transform the pose to the target frame (e.g., the planning frame)
				// Use a timeout (e.g., 1 second) to wait for the transform
				final_pose_msg = tf_buffer_->transform(target_pose_msg, target_frame_, tf2::durationFromSec(1.0));
				RCLCPP_DEBUG(getLogger(), "Transformed grasp pose from '%s' to '%s'",
				            target_pose_msg.header.frame_id.c_str(), target_frame_.c_str());
			} catch (const tf2::TransformException& ex) {
				RCLCPP_ERROR(getLogger(), "Failed to transform grasp pose from '%s' to '%s': %s",
				             target_pose_msg.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
				continue; // Skip this grasp if transform fails
			}
		} else {
			if (target_pose_msg.header.frame_id.empty()){
				RCLCPP_WARN(getLogger(), "Input grasp pose has empty frame_id. Assuming it's already in the target frame '%s'.", target_frame_.c_str());
				final_pose_msg = target_pose_msg;
				final_pose_msg.header.frame_id = target_frame_;
			} else if (target_pose_msg.header.frame_id != target_frame_){
				RCLCPP_WARN(getLogger(), "TF buffer not available or target frame not set. Using original grasp frame '%s'.",
				            target_pose_msg.header.frame_id.c_str());
				final_pose_msg = target_pose_msg;
			} else {
				// Frame IDs already match or no transform needed/possible
				final_pose_msg = target_pose_msg;
			}
		}

		// Create an InterfaceState. Generator likely needs a base scene.
		// Let's assume the current scene is the intended base.
		moveit::task_constructor::InterfaceState state(scene);
		state.properties().set("target_pose", final_pose_msg);



		// Use spawn() as required by Generator, passing the grasp score as the cost
		spawn(std::move(state), (700.0 - static_cast<double>(grasp.score.data))/100.0);
	}

	generated_ = true; // Set the flag after generating poses
}

void GraspProvider::onNewSolution(const moveit::task_constructor::SolutionBase& solution)
{
	// It's safe to store a pointer to this solution, as the generating stage stores it
	upstream_solutions_.push(&solution);
	RCLCPP_INFO(getLogger(), "Received new solution with score: %f", solution.cost());
}

} // namespace moveit_task_constructor_demo 