#include <string>
#include <array>
#include <lcm/lcm-cpp.hpp>

// ROS Hardware Interface headers
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// ROS message headers
#include "victor_hardware_interface/ControlModeParameters.h"
#include "victor_hardware_interface/MotionCommand.h"
#include "victor_hardware_interface/MotionStatus.h"
// LCM type headers
#include "victor_hardware_interface/control_mode_parameters.hpp"
#include "victor_hardware_interface/motion_command.hpp"
#include "victor_hardware_interface/motion_status.hpp"

#ifndef IIWA_HARDWARE_INTERFACE_HPP
#define IIWA_HARDWARE_INTERFACE_HPP

constexpr auto const joint_names = std::array<char const *, 7>{
        "",
        "",
        "",
        "",
        "",
        "",
        "",
};
constexpr auto const n_joints = joint_names.size();

namespace victor_hardware_interface {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LCM to ROS and ROS to LCM message converters
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    JointValueQuantity jvqLcmToRos(const joint_value_quantity &lcm_jvq);

    joint_value_quantity jvqRosToLcm(const JointValueQuantity &ros_jvq);

    CartesianValueQuantity cvqLcmToRos(const cartesian_value_quantity &lcm_cvq);

    cartesian_value_quantity cvqRosToLcm(const CartesianValueQuantity &ros_cvq);

    geometry_msgs::Pose poseLcmToRos(const cartesian_pose &lcm_pose);

    cartesian_pose poseRosToLcm(const geometry_msgs::Pose &ros_pose);

    JointImpedanceParameters jointImpedanceParamsLcmToRos(const joint_impedance_parameters &lcm_jip);

    joint_impedance_parameters jointImpedanceParamsRosToLcm(const JointImpedanceParameters &ros_jip);

    CartesianImpedanceParameters cartesianImpedanceParamsLcmToRos(const cartesian_impedance_parameters &lcm_cip);

    cartesian_impedance_parameters cartesianImpedanceParamsRosToLcm(const CartesianImpedanceParameters &ros_cip);

    JointPathExecutionParameters jointPexpLcmToRos(const joint_path_execution_parameters &path_execution_params);

    joint_path_execution_parameters jointPexpRosToLcm(const JointPathExecutionParameters &path_execution_params);

    CartesianPathExecutionParameters cartesianPexpLcmToRos(const cartesian_path_execution_parameters &lcm_pexp);

    cartesian_path_execution_parameters cartesianPexpRosToLcm(const CartesianPathExecutionParameters &ros_pexp);

    CartesianControlModeLimits cartesianControlModeLimitsLcmToRos(const cartesian_control_mode_limits &lcm_ccml);

    cartesian_control_mode_limits cartesianControlModeLimitsRosToLcm(const CartesianControlModeLimits &ros_ccml);

    ControlMode controlModeLcmToRos(const control_mode &lcm_cm);

    control_mode controlModeRosToLcm(const ControlMode &ros_cm);

    MotionStatus motionStatusLcmToRos(const motion_status &lcm_status);

    motion_command motionCommandRosToLcm(const MotionCommand &ros_command);

    ControlModeParameters controlModeParamsLcmToRos(const control_mode_parameters &lcm_cmp);

    control_mode_parameters controlModeParamsRosToLcm(const ControlModeParameters &ros_cmp);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The class that does the actual communication
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class IIWAHardwareInterface : public hardware_interface::RobotHW {
    protected:

        std::shared_ptr<lcm::LCM> send_lcm_ptr_;
        std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
        std::string motion_command_channel_name_;
        std::string motion_status_channel_name_;
        std::function<void(const MotionStatus &)> motion_status_callback_fn_;
        std::string control_mode_command_channel_name_;
        std::string control_mode_status_channel_name_;
        std::function<void(const ControlModeParameters &)> control_mode_status_callback_fn_;
        MotionStatus latest_motion_status_msg_;
        MotionStatus latest_gripper_status_msg_;
        MotionCommand latest_motion_command_msg_;
        MotionCommand latest_gripper_command_msg_;

        void InternalMotionStatusLCMCallback(
                const lcm::ReceiveBuffer *buffer,
                const std::string &channel,
                const motion_status *status_msg);

        void InternalControlModeStatusLCMCallback(
                const lcm::ReceiveBuffer *buffer,
                const std::string &channel,
                const control_mode_parameters *status_msg);

    public:

        IIWAHardwareInterface(
                const std::shared_ptr<lcm::LCM> &send_lcm_ptr,
                const std::shared_ptr<lcm::LCM> &recv_lcm_ptr,
                const std::string &motion_command_channel_name,
                const std::string &motion_status_channel_name,
                const std::function<void(const MotionStatus &)> &motion_status_callback_fn,
                const std::string &control_mode_command_channel_name,
                const std::string &control_mode_status_channel_name,
                const std::function<void(const ControlModeParameters &)> &control_mode_status_callback_fn);

        bool SendMotionCommandMessage(const MotionCommand &command);

        bool SendControlModeCommandMessage(const ControlModeParameters &command);

        // For explanation, see https://github.com/ros-controls/ros_control/wiki/hardware_interface
        // these are somewhat duplicative of the other functions in this class
        void read(ros::Time const &time, ros::Duration const &period) override;

        void write(ros::Time const &time, ros::Duration const &period) override;


    private:
        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::PositionJointInterface joint_pos_interface;
        double cmd[n_joints];
        double pos[n_joints];
        double vel[n_joints];
        double eff[n_joints];
    };
}

#endif // IIWA_HARDWARE_INTERFACE_HPP
