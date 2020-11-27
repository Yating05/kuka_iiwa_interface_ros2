## Usage

    roslaunch victor_fake_hardware_interface fake_dual_arm_lcm_bridge.launch talkative:=false

If you also want planning, you'll need to run the move group

    ROS_NAMESPACE=victor roslaunch victor_moveit_config move_group.launch allow_trajectory_execution:=false

If you want "fake" execution via moveit, you also need the trajectory follower from `arm_robots`

    roslaunch arm_robots trajectory_follower.launch use_victor:=true
    ROS_NAMESPACE=victor roslaunch victor_moveit_config move_group.launch
