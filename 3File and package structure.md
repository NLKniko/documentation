# File and package structure
In this section we want to explain the structure of the packages which provided by Trossen Robotics.

```
.
├── create3_sim_ros1
│   ├── irobot_create_common
│   │   ├── irobot_create_common_bringup
│   │   ├── irobot_create_control
│   │   ├── irobot_create_description
│   │   ├── irobot_create_nodes
│   │   └── irobot_create_toolbox
│   ├── irobot_create_gazebo
│   ├── irobot_create_ignition
│   │   ├── irobot_create_ignition_plugins
│   │   ├── irobot_create_ignition_sim
│   │   └── irobot_create_ignition_toolbox
│   ├── LICENSE
│   └── README.md
|
├── interbotix_ros_core
│   ├── images
│   ├── interbotix_ros_uxarms
│   └── interbotix_ros_xseries
|
├── interbotix_ros_rovers
│   ├── examples
│   │   ├── interbotix_xslocobot_joy
│   │   │   ├── config
│   │   │   │   ├── modes_all.yaml
│   │   │   │   └── modes_base.yaml
│   │   │   ├── images
│   │   │   │   ├── ps3.jpg
│   │   │   │   ├── ps4.jpg
│   │   │   │   └── xslocobot_joy_flowchart.png
│   │   │   ├── launch
│   │   │   │   └── xslocobot_joy.launch
│   │   │   ├── scripts
│   │   │   │   └── xslocobot_robot
│   │   │   └── src
│   │   │       └── xslocobot_joy.cpp
│   │   ├── interbotix_xslocobot_landmark_nav
│   │   │   ├── landmarks
│   │   │   │   └── landmarks.yaml
│   │   │   ├── launch
│   │   │   │   ├── landmark_manager.launch
│   │   │   │   └── xslocobot_landmark_nav.launch
│   │   │   ├── scripts
│   │   │   │   └── nav_to_landmark
│   │   │   └── setup.py
│   │   ├── interbotix_xslocobot_moveit_interface
│   │   │   ├── config
│   │   │   │   ├── locobot_px100.yaml
│   │   │   │   ├── locobot_wx200.yaml
│   │   │   │   ├── locobot_wx250s.yaml
│   │   │   │   └── modes_all.yaml
│   │   │   └── launch
│   │   │       └── xslocobot_moveit_interface.launch
│   │   └── python_demos
│   │       ├── auto_docking.py
│   │       ├── bartender.py
│   │       ├── combo_control.py
│   │       ├── ee_cartesian_trajectory.py
│   │       ├── ee_pose_components.py
│   │       ├── ee_pose_matrix_control.py
│   │       ├── gripper_control.py
│   │       ├── joint_position_control.py
│   │       ├── joint_pwm_control.py
│   │       ├── move_base.py
│   │       └── pan_tilt_control.py
│   ├── interbotix_ros_xslocobots
│   │   ├── examples
│   │   │   ├── interbotix_xslocobot_joy
│   │   │   │   ├── config
│   │   │   │   │   ├── modes_all.yaml
│   │   │   │   │   └── modes_base.yaml
│   │   │   │   ├── launch
│   │   │   │   │   └── xslocobot_joy.launch
│   │   │   │   ├── scripts
│   │   │   │   │   └── xslocobot_robot
│   │   │   │   └── src
│   │   │   │       └── xslocobot_joy.cpp
│   │   │   ├── interbotix_xslocobot_landmark_nav
│   │   │   │   ├── landmarks
│   │   │   │   │   └── landmarks.yaml
│   │   │   │   ├── launch
│   │   │   │   │   ├── landmark_manager.launch
│   │   │   │   │   └── xslocobot_landmark_nav.launch
│   │   │   │   ├── scripts
│   │   │   │   │   └── nav_to_landmark
│   │   │   │   └── setup.py
│   │   │   ├── interbotix_xslocobot_moveit_interface
│   │   │   │   ├── config
│   │   │   │   │   ├── locobot_px100.yaml
│   │   │   │   │   ├── locobot_wx200.yaml
│   │   │   │   │   ├── locobot_wx250s.yaml
│   │   │   │   │   └── modes_all.yaml
│   │   │   │   └── launch
│   │   │   │       └── xslocobot_moveit_interface.launch
│   │   │   └── python_demos
│   │   │       ├── auto_docking.py
│   │   │       ├── bartender.py
│   │   │       ├── combo_control.py
│   │   │       ├── ee_cartesian_trajectory.py
│   │   │       ├── ee_pose_components.py
│   │   │       ├── ee_pose_matrix_control.py
│   │   │       ├── gripper_control.py
│   │   │       ├── joint_position_control.py
│   │   │       ├── joint_pwm_control.py
│   │   │       ├── move_base.py
│   │   │       └── pan_tilt_control.py
│   │   ├── interbotix_xslocobot_nav
│   │   │   ├── config
│   │   │   │   ├── common_costmap_params.yaml
│   │   │   │   ├── global_costmap_all_params.yaml
│   │   │   │   ├── global_costmap_depth_params.yaml
│   │   │   │   ├── global_planner_params.yaml
│   │   │   │   ├── local_costmap_all_params.yaml
│   │   │   │   ├── local_costmap_depth_params.yaml
│   │   │   │   ├── local_planner_params.yaml
│   │   │   │   └── move_base_params.yaml
│   │   │   └── launch
│   │   │       ├── xslocobot_nav.launch
│   │   │       └── xslocobot_nav_sim.launch
│   │   ├── interbotix_xslocobot_perception
│   │   │   ├── config
│   │   │   │   └── filter_params.yaml
│   │   │   ├── launch
│   │   │   │   └── xslocobot_perception.launch
│   │   │   └── scripts
│   │   │       ├── pick_place_armtag.py
│   │   │       └── pick_place_no_armtag.py
│   ├── interbotix_xslocobot_control
│   │   ├── config
│   │   │   ├── bridge.yaml
│   │   │   ├── locobot_base.yaml
│   │   │   ├── locobot_px100.yaml
│   │   │   ├── locobot_wx200.yaml
│   │   │   ├── locobot_wx250s.yaml
│   │   │   ├── modes_all.yaml
│   │   │   ├── modes_base.yaml
│   │   │   └── tf_rebroadcaster.yaml
│   │   ├── launch
│   │   │   ├── xslocobot_control.launch
│   │   │   └── xslocobot_python.launch
│   │   └── scripts
│   │       └── startup_bridge.sh
│   ├── interbotix_xslocobot_descriptions
│   │   ├── launch
│   │   │   ├── many_xslocobots.launch
│   │   │   ├── remote_view.launch
│   │   │   ├── rtabmapviz_remote.launch
│   │   │   └── xslocobot_description.launch
│   │   ├── meshes
│   │   │   ├── locobot_meshes
│   │   │   │   ├── create3_version
│   │   │   │   ├── kobuki_version
│   │   │   │   ├── locobot_arm_cradle.stl
│   │   │   │   ├── locobot_battery.stl
│   │   │   │   ├── locobot_camera.stl
│   │   │   │   ├── locobot_lidar.stl
│   │   │   │   ├── locobot_lidar_tower_simple.stl
│   │   │   │   ├── locobot_lidar_tower.stl
│   │   │   │   ├── locobot_pan.stl
│   │   │   │   └── locobot_tilt.stl
│   │   │   ├── mobile_px100_meshes
│   │   │   │   ├── mobile_px100_1_base.stl
│   │   │   │   ├── mobile_px100_2_shoulder.stl
│   │   │   │   ├── mobile_px100_3_upper_arm.stl
│   │   │   │   ├── mobile_px100_4_forearm.stl
│   │   │   │   ├── mobile_px100_5_gripper.stl
│   │   │   │   ├── mobile_px100_6_gripper_prop.stl
│   │   │   │   ├── mobile_px100_7_gripper_bar.stl
│   │   │   │   ├── mobile_px100_8_gripper_finger.stl
│   │   │   │   └── mobile_px100_9_ar_tag.stl
│   │   │   ├── mobile_wx200_meshes
│   │   │   │   ├── mobile_wx200_10_ar_tag.stl
│   │   │   │   ├── mobile_wx200_1_base.stl
│   │   │   │   ├── mobile_wx200_2_shoulder.stl
│   │   │   │   ├── mobile_wx200_3_upper_arm.stl
│   │   │   │   ├── mobile_wx200_4_forearm.stl
│   │   │   │   ├── mobile_wx200_5_wrist.stl
│   │   │   │   ├── mobile_wx200_6_gripper.stl
│   │   │   │   ├── mobile_wx200_7_gripper_prop.stl
│   │   │   │   ├── mobile_wx200_8_gripper_bar.stl
│   │   │   │   └── mobile_wx200_9_gripper_finger.stl
│   │   │   └── mobile_wx250s_meshes
│   │   │       ├── mobile_wx250s_10_gripper_finger.stl
│   │   │       ├── mobile_wx250s_11_ar_tag.stl
│   │   │       ├── mobile_wx250s_1_base.stl
│   │   │       ├── mobile_wx250s_2_shoulder.stl
│   │   │       ├── mobile_wx250s_3_upper_arm.stl
│   │   │       ├── mobile_wx250s_4_upper_forearm.stl
│   │   │       ├── mobile_wx250s_5_lower_forearm.stl
│   │   │       ├── mobile_wx250s_6_wrist.stl
│   │   │       ├── mobile_wx250s_7_gripper.stl
│   │   │       ├── mobile_wx250s_8_gripper_prop.stl
│   │   │       └── mobile_wx250s_9_gripper_bar.stl
│   │   ├── rviz
│   │   │   ├── many_locobots.rviz
│   │   │   └── xslocobot_description.rviz
│   │   └── urdf
│   │       ├── arm_cradle.urdf.xacro
│   │       ├── arms
│   │       │   ├── mobile_arm.urdf.xacro
│   │       │   ├── mobile_px100.urdf.xacro
│   │       │   ├── mobile_wx200.urdf.xacro
│   │       │   └── mobile_wx250s.urdf.xacro
│   │       ├── battery.urdf.xacro
│   │       ├── camera_tower.urdf.xacro
│   │       ├── create3_version
│   │       │   ├── bump_caster.urdf.xacro
│   │       │   ├── bumper.urdf.xacro
│   │       │   ├── button.urdf.xacro
│   │       │   ├── caster.urdf.xacro
│   │       │   ├── common_properties.urdf.xacro
│   │       │   ├── dock
│   │       │   │   ├── ir_emitter.urdf.xacro
│   │       │   │   └── standard_dock.urdf.xacro
│   │       │   ├── locobot_create3.urdf.xacro
│   │       │   ├── sensors
│   │       │   │   ├── cliff_sensor.urdf.xacro
│   │       │   │   ├── imu.urdf.xacro
│   │       │   │   ├── ir_intensity.urdf.xacro
│   │       │   │   ├── ir_opcode_receivers.urdf.xacro
│   │       │   │   └── optical_mouse.urdf.xacro
│   │       │   ├── wheel_drop.urdf.xacro
│   │       │   ├── wheel.urdf.xacro
│   │       │   └── wheel_with_wheeldrop.urdf.xacro
│   │       ├── kobuki_version
│   │       │   └── locobot_kobuki.urdf.xacro
│   │       ├── lidar.urdf.xacro
│   │       ├── locobot.urdf.xacro
│   │       ├── pan_and_tilt.urdf.xacro
│   │       └── plate.urdf.xacro
│   ├── interbotix_xslocobot_gazebo
│   │   ├── config
│   │   │   ├── locobot_gazebo_controllers.yaml
│   │   │   ├── position_controllers
│   │   │   │   ├── mobile_px100_position_controllers.yaml
│   │   │   │   ├── mobile_wx200_position_controllers.yaml
│   │   │   │   └── mobile_wx250s_position_controllers.yaml
│   │   │   └── trajectory_controllers
│   │   │       ├── mobile_px100_trajectory_controllers.yaml
│   │   │       ├── mobile_wx200_trajectory_controllers.yaml
│   │   │       └── mobile_wx250s_trajectory_controllers.yaml
│   │   ├── launch
│   │   │   └── xslocobot_gazebo.launch
│   │   ├── media
│   │   │   ├── materials
│   │   │   │   ├── scripts
│   │   │   │   │   └── interbotix_black.material
│   │   │   │   └── textures
│   │   │   └── models
│   │   │       └── TrossenRoboticsOfficeBuilding
│   │   │           ├── model.config
│   │   │           └── TrossenRoboticsOfficeBuilding.sdf
│   │   └── worlds
│   │       └── xslocobot_gazebo.world
│   ├── interbotix_xslocobot_moveit
│   │   ├── config
│   │   │   ├── chomp_planning.yaml
│   │   │   ├── controllers
│   │   │   │   ├── 4dof_controllers.yaml
│   │   │   │   ├── 5dof_controllers.yaml
│   │   │   │   └── 6dof_controllers.yaml
│   │   │   ├── fake_controllers
│   │   │   │   ├── 4dof_controllers.yaml
│   │   │   │   ├── 5dof_controllers.yaml
│   │   │   │   └── 6dof_controllers.yaml
│   │   │   ├── joint_limits
│   │   │   │   ├── 4dof_joint_limits.yaml
│   │   │   │   ├── 5dof_joint_limits.yaml
│   │   │   │   └── 6dof_joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── modes_all.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   ├── sensors_3d.yaml
│   │   │   └── srdf
│   │   │       ├── locobot_px100.srdf.xacro
│   │   │       ├── locobot_wx200.srdf.xacro
│   │   │       └── locobot_wx250s.srdf.xacro
│   │   └── launch
│   │       ├── chomp_planning_pipeline.launch.xml
│   │       ├── default_warehouse_db.launch
│   │       ├── joystick_control.launch
│   │       ├── move_group.launch
│   │       ├── moveit.rviz
│   │       ├── moveit_rviz.launch
│   │       ├── ompl_planning_pipeline.launch.xml
│   │       ├── planning_context.launch
│   │       ├── planning_pipeline.launch.xml
│   │       ├── run_benchmark_ompl.launch
│   │       ├── sensor_manager.launch.xml
│   │       ├── setup_assistant.launch
│   │       ├── trajectory_execution.launch.xml
│   │       ├── warehouse.launch
│   │       ├── warehouse_settings.launch.xml
│   │       ├── xslocobot_moveit_controller_manager.launch.xml
│   │       ├── xslocobot_moveit.launch
│   │       └── xslocobot_moveit_sensor_manager.launch.xml
│   ├── interbotix_xslocobot_nav
│   │   ├── config
│   │   │   ├── common_costmap_params.yaml
│   │   │   ├── global_costmap_all_params.yaml
│   │   │   ├── global_costmap_depth_params.yaml
│   │   │   ├── global_planner_params.yaml
│   │   │   ├── local_costmap_all_params.yaml
│   │   │   ├── local_costmap_depth_params.yaml
│   │   │   ├── local_planner_params.yaml
│   │   │   └── move_base_params.yaml
│   │   └── launch
│   │       ├── xslocobot_nav.launch
│   │       └── xslocobot_nav_sim.launch
│   ├── interbotix_xslocobot_perception
│   │   ├── config
│   │   │   └── filter_params.yaml
│   │   ├── launch
│   │   │   └── xslocobot_perception.launch
│   │   └── scripts
│   │       ├── pick_place_armtag.py
│   │       └── pick_place_no_armtag.py
│   └── interbotix_xslocobot_ros_control
│       ├── config
│       │   ├── 4dof_controllers.yaml
│       │   ├── 5dof_controllers.yaml
│       │   ├── 6dof_controllers.yaml
│       │   ├── hardware.yaml
│       │   └── modes_all.yaml
│       └── launch
|           └── xslocobot_ros_control.launch
|
└── interbotix_ros_toolboxes
    ├── interbotix_common_toolbox
    │   ├── interbotix_common_modules
    │   │   ├── setup.py
    │   │   └── src
    │   │       └── interbotix_common_modules
    │   │           ├── angle_manipulation.py
    │   │           ├── geometry.py
    │   │           └── __init__.py
    │   ├── interbotix_landmark_modules
    │   │   ├── landmarks
    │   │   │   └── landmarks.yaml
    │   │   ├── launch
    │   │   │   ├── landmark.launch
    │   │   │   ├── landmark_manager.launch
    │   │   │   └── tf_map_to_landmark.launch
    │   │   ├── scripts
    │   │   │   ├── landmark_finder
    │   │   │   ├── landmark_manager
    │   │   │   └── tf_map_to_landmark
    │   │   ├── setup.py
    │   │   ├── src
    │   │   │   └── interbotix_landmark_modules
    │   │   │       ├── __init__.py
    │   │   │       └── landmark.py
    │   │   └── test
    │   │       ├── test_landmark.py
    │   │       └── test-landmark.test
    │   ├── interbotix_moveit_interface
    │   │   ├── include
    │   │   │   └── interbotix_moveit_interface
    │   │   │       └── moveit_interface_obj.h
    │   │   ├── scripts
    │   │   │   ├── moveit_interface_gui
    │   │   │   └── moveit_python_interface
    │   │   ├── src
    │   │   │   ├── moveit_interface.cpp
    │   │   │   └── moveit_interface_obj.cpp
    │   │   └── srv
    │   │       └── MoveItPlan.srv
    │   └── interbotix_tf_tools
    │       ├── config
    │       │   └── tf_rebroadcaster.yaml
    │       ├── include
    │       │   └── interbotix_tf_tools
    │       │       └── tf_rebroadcaster.h
    │       ├── launch
    │       │   └── tf_rebroadcaster.launch
    │       └── src
    │           └── tf_rebroadcaster.cpp
    ├── interbotix_perception_toolbox
    │   └── interbotix_perception_modules
    │       ├── config
    │       │   └── tags.yaml
    │       ├── launch
    │       │   ├── apriltag.launch
    │       │   ├── armtag.launch
    │       │   ├── pc_filter.launch
    │       │   ├── picture_snapper.launch
    │       │   └── static_transform_pub.launch
    │       ├── msg
    │       │   └── ClusterInfo.msg
    │       ├── scripts
    │       │   ├── armtag_tuner_gui
    │       │   ├── picture_snapper
    │       │   ├── pointcloud_tuner_gui
    │       │   └── static_trans_pub
    │       ├── setup.py
    │       ├── src
    │       │   ├── interbotix_perception_modules
    │       │   │   ├── apriltag.py
    │       │   │   ├── armtag.py
    │       │   │   ├── __init__.py
    │       │   │   └── pointcloud.py
    │       │   ├── perception_pipeline.cpp
    │       │   └── ui
    │       │       ├── armtagtunergui.ui
    │       │       └── pointcloudtunergui.ui
    │       └── srv
    │           ├── ClusterInfoArray.srv
    │           ├── FilterParams.srv
    │           └── SnapPicture.srv
    ├── interbotix_rpi_toolbox
    │   └── interbotix_rpi_modules
    │       ├── msg
    │       │   └── PixelCommands.msg
    │       ├── scripts
    │       │   └── rpi_pixels
    │       ├── setup.py
    │       └── src
    │           └── interbotix_rpi_modules
    │               ├── __init__.py
    │               └── neopixels.py
    ├── interbotix_ux_toolbox
    │   ├── interbotix_ux_modules
    │   │   ├── setup.py
    │   │   └── src
    │   │       └── interbotix_ux_modules
    │   │           ├── arm.py
    │   │           ├── core.py
    │   │           ├── gripper.py
    │   │           ├── __init__.py
    │   │           └── mr_descriptions.py
    │   └── interbotix_ux_ros_control
    │       ├── include
    │       │   └── interbotix_ux_ros_control
    │       │       └── ux_hardware_interface_obj.h
    │       └── src
    │           ├── ux_hardware_interface.cpp
    │           └── ux_hardware_interface_obj.cpp
    ├── interbotix_xs_toolbox
    │   ├── interbotix_xs_modules
    │   │   ├── setup.py
    │   │   └── src
    │   │       └── interbotix_xs_modules
    │   │           ├── arm.py
    │   │           ├── core.py
    │   │           ├── create3.py
    │   │           ├── gripper.py
    │   │           ├── hexapod.py
    │   │           ├── __init__.py
    │   │           ├── InterbotixArmXSInterface.m
    │   │           ├── InterbotixGripperXSInterface.m
    │   │           ├── InterbotixGripperXS.m
    │   │           ├── InterbotixManipulatorXS.m
    │   │           ├── InterbotixRobotXSCore.m
    │   │           ├── kobuki.py
    │   │           ├── locobot.py
    │   │           ├── mr_descriptions.m
    │   │           ├── mr_descriptions.py
    │   │           └── turret.py
    │   ├── interbotix_xs_ros_control
    │   │   ├── include
    │   │   │   └── interbotix_xs_ros_control
    │   │   │       └── xs_hardware_interface_obj.h
    │   │   └── src
    │   │       ├── xs_hardware_interface.cpp
    │   │       └── xs_hardware_interface_obj.cpp
    │   └── interbotix_xs_rviz
    │       ├── include
    │       │   └── interbotix_xs_rviz
    │       │       ├── interbotix_control_panel.hpp
    │       │       └── xs_register_descriptions.h
    │       └── src
    │           ├── interbotix_control_panel.cpp
    │           └── ui
    │               └── interbotix_control_panel.ui
    └── third_party_libraries
        ├── ModernRobotics
        └── README.md

```