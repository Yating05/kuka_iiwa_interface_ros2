cmake_minimum_required(VERSION 2.8.3)
project(victor_hardware_interface_msgs)

set(CATKIN_PACKAGES
    diagnostic_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation)
find_package(cmake_modules REQUIRED)

catkin_python_setup()

add_message_files(DIRECTORY msg FILES
    CartesianControlModeLimits.msg
    CartesianImpedanceParameters.msg
    CartesianPathExecutionParameters.msg
    CartesianValueQuantity.msg
    ControlMode.msg
    ControlModeParameters.msg
    JointImpedanceParameters.msg
    JointPathExecutionParameters.msg
    JointValueQuantity.msg
    MotionCommand.msg
    MotionStatus.msg
    Robotiq3FingerActuatorCommand.msg
    Robotiq3FingerActuatorStatus.msg
    Robotiq3FingerCommand.msg
    Robotiq3FingerObjectStatus.msg
    Robotiq3FingerStatus.msg
    GraspStatus.msg)

add_service_files(DIRECTORY srv FILES
    SetControlMode.srv
    GetControlMode.srv)

generate_messages(DEPENDENCIES
    diagnostic_msgs
    geometry_msgs
    sensor_msgs
    std_msgs)

catkin_package(
    LIBRARIES       ${PROJECT_NAME} 
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
                    message_runtime)
