cmake_minimum_required(VERSION 3.16)
project(moveit_task_constructor_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(grasp_msgs REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
	rclcpp
	tf2_eigen
	grasp_msgs
)

add_library(moveit_task_constructor_demo_pick_place_task
  src/pick_place_task.cpp
  src/grasp_provider.cpp
)
target_include_directories(moveit_task_constructor_demo_pick_place_task
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(moveit_task_constructor_demo_pick_place_task
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_common
  tf2_eigen
  ${moveit_task_constructor_demo_parameters_TARGETS}
  grasp_msgs
)

include_directories(include)

# declare a demo consisting of a single cpp file
function(demo name)
	add_executable(${PROJECT_NAME}_${name} src/${name}.cpp)
	set(parameter_filename ${name}_parameters)
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${parameter_filename}.yaml)
		generate_parameter_library(${parameter_filename} src/${parameter_filename}.yaml)
		target_link_libraries(${PROJECT_NAME}_${name} ${parameter_filename})
	endif()
	ament_target_dependencies(${PROJECT_NAME}_${name} ${THIS_PACKAGE_INCLUDE_DEPENDS})
	set_target_properties(${PROJECT_NAME}_${name} PROPERTIES OUTPUT_NAME ${name} PREFIX "")
	install(TARGETS ${PROJECT_NAME}_${name}
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION lib/${PROJECT_NAME}
	)
endfunction()

demo(cartesian)
demo(modular)
demo(alternative_path_costs)
demo(ik_clearance_cost)
demo(fallbacks_move_to)

demo(pick_place_demo)
target_link_libraries(${PROJECT_NAME}_pick_place_task pick_place_demo_parameters)
target_link_libraries(${PROJECT_NAME}_pick_place_demo ${PROJECT_NAME}_pick_place_task)

install(PROGRAMS
        scripts/alternatives.py
        scripts/cartesian.py
        scripts/compute_ik.py
        scripts/constrained.py
        scripts/current_state.py
        scripts/fallbacks.py
        scripts/fix_collision_objects.py
        scripts/fixed_state.py
        scripts/generate_pose.py
        scripts/merger.py
        scripts/modify_planning_scene.py
        scripts/multi_planner.py
        scripts/pickplace.py
        scripts/properties.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

# TODO: Port tests
# add_subdirectory(test)

ament_package()
