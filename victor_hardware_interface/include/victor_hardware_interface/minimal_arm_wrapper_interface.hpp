#include <thread>
#include <string>
#include <mutex>
#include <arc_utilities/maybe.hpp>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
// ROS message headers
#include <victor_hardware_interface/ControlModeCommand.h>
#include <victor_hardware_interface/ControlModeStatus.h>
#include <victor_hardware_interface/MotionCommand.h>
#include <victor_hardware_interface/MotionStatus.h>
#include <victor_hardware_interface/Robotiq3FingerCommand.h>
#include <victor_hardware_interface/Robotiq3FingerStatus.h>
#include <victor_hardware_interface/SetControlMode.h>
#include <victor_hardware_interface/GetControlMode.h>

// LCM
#include <lcm/lcm-cpp.hpp>

// Classes to speak to each invididual hardware element
#include <victor_hardware_interface/iiwa_hardware_interface.hpp>
#include <victor_hardware_interface/robotiq_3finger_hardware_interface.hpp>

#ifndef MINIMAL_ARM_WRAPPER_INTERFACE_HPP
#define MINIMAL_ARM_WRAPPER_INTERFACE_HPP

namespace victor_hardware_interface
{
    class MinimalArmWrapperInterface
    {
    protected:

        ros::NodeHandle nh_;
        const std::string cartesian_control_frame_;
        ros::Publisher motion_status_pub_;
        ros::Publisher control_mode_status_pub_;
        ros::Publisher gripper_status_pub_;
        ros::Subscriber motion_command_sub_;
        ros::Subscriber gripper_command_sub_;
        ros::ServiceServer set_control_mode_server_;
        ros::ServiceServer get_control_mode_server_;
        ros::CallbackQueue ros_callback_queue_;
        std::thread ros_callback_thread_;

        Maybe::Maybe<victor_hardware_interface::ControlModeStatus> active_control_mode_;
        std::mutex control_mode_status_mutex_;
        const double set_control_mode_timeout_;

        std::shared_ptr<lcm::LCM> send_lcm_ptr_;
        std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
        std::unique_ptr<iiwa_hardware_interface::IIWAHardwareInterface> iiwa_ptr_;
        std::unique_ptr<robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface> robotiq_ptr_;

    public:

        MinimalArmWrapperInterface(
                ros::NodeHandle& nh,
                const std::shared_ptr<lcm::LCM>& send_lcm_ptr,
                const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
                const std::string& cartesian_control_frame,
                const double set_control_mode_timeout,
                // ROS Topics
                const std::string& motion_command_topic,
                const std::string& motion_status_topic,
                const std::string& control_mode_status_topic,
                const std::string& get_control_mode_service,
                const std::string& set_control_mode_service,
                const std::string& gripper_command_topic,
                const std::string& gripper_status_topic,
                // LCM channels
                const std::string& motion_command_channel,
                const std::string& motion_status_channel,
                const std::string& control_mode_command_channel,
                const std::string& control_mode_status_channel,
                const std::string& gripper_command_channel,
                const std::string& gripper_status_channel);

        void ROSCallbackThread();

        void LCMLoop();

        static bool JVQMatch(const victor_hardware_interface::JointValueQuantity& jvq1, const victor_hardware_interface::JointValueQuantity& jvq2);

        static bool CVQMatch(const victor_hardware_interface::CartesianValueQuantity& cvq1, const victor_hardware_interface::CartesianValueQuantity& cvq2);

        static bool jointPExPMatch(const victor_hardware_interface::JointPathExecutionParameters& pexp1, const victor_hardware_interface::JointPathExecutionParameters& pexp2);

        static bool cartesianPExPMatch(const victor_hardware_interface::CartesianPathExecutionParameters& pexp1, const victor_hardware_interface::CartesianPathExecutionParameters& pexp2);

        static bool checkControlModeCommandAndStatusMatch(const victor_hardware_interface::ControlModeCommand& command, const victor_hardware_interface::ControlModeStatus& status);

        std::pair<bool, std::string> safetyCheckJointPathExecutionParams(const victor_hardware_interface::JointPathExecutionParameters& params) const;

        std::pair<bool, std::string> safetyCheckCartesianPathExecutionParams(const victor_hardware_interface::CartesianPathExecutionParameters& params) const;

        std::pair<bool, std::string> safetyCheckJointImpedanceParams(const victor_hardware_interface::JointImpedanceParameters& params) const;

        std::pair<bool, std::string> safetyCheckCartesianImpedanceParams(const victor_hardware_interface::CartesianImpedanceParameters& params) const;

        std::pair<bool, std::string> safetyCheckCartesianControlModeLimits(const victor_hardware_interface::CartesianControlModeLimits& params) const;

        std::pair<bool, std::string> safetyCheckControlMode(const victor_hardware_interface::ControlModeCommand& control_mode) const;

        bool setControlModeCallback(victor_hardware_interface::SetControlMode::Request& req, victor_hardware_interface::SetControlMode::Response& res);

        bool getControlModeCallback(victor_hardware_interface::GetControlMode::Request& req, victor_hardware_interface::GetControlMode::Response& res);

        bool safetyCheckPositions(const victor_hardware_interface::JointValueQuantity& positions) const;

        bool safetyCheckPositionsVelocities(const victor_hardware_interface::JointValueQuantity& positions, const victor_hardware_interface::JointValueQuantity& velocities) const;

        bool safetyCheckCartesianPose(const geometry_msgs::Pose& pose, const std::string& frame) const;

        bool safetyCheckMotionCommand(const victor_hardware_interface::MotionCommand& command);

        void motionCommandROSCallback(victor_hardware_interface::MotionCommand command);

        bool safetyCheckFingerCommand(const victor_hardware_interface::Robotiq3FingerActuatorCommand& command) const;

        bool safetyCheckGripperCommand(const victor_hardware_interface::Robotiq3FingerCommand& command) const;

        void gripperCommandROSCallback(victor_hardware_interface::Robotiq3FingerCommand command);

        void motionStatusLCMCallback(const victor_hardware_interface::MotionStatus& motion_status);

        void controlModeStatusLCMCallback(const victor_hardware_interface::ControlModeStatus& control_mode_status);

        void gripperStatusLCMCallback(const victor_hardware_interface::Robotiq3FingerStatus& gripper_status);

    };
}

#endif // MINIMAL_ARM_WRAPPER_INTERFACE_HPP
