//  Copyright (c) 2026 KU Leuven, Belgium
//
//  Author: Santiago Iregui
//  email: <santiago.iregui@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_interfacing_utils/robotdriver.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"


namespace etasl {

/**
 * @brief Generic crospi robot driver for any robot that has a ros2_control hardware interface.
 *
 * The robot description (URDF/xacro) given in the configuration is loaded in a hardware_interface::ResourceManager,
 * which loads the hardware components (ros2_control hardware interface plugins) declared in its <ros2_control> tags.
 * There is no controller_manager and there are no controllers: the real-time thread of crospi that executes
 * update() performs the read() -> crospi -> write() cycle of the hardware directly. The joint velocity setpoints of
 * crospi are sent to the velocity command interfaces of the joints, or integrated into position commands.
 *
 * The same class is registered both as robotdriver and as robotsimulator (see plugins.xml), so that simulated
 * hardware (mock_components/GenericSystem, topic_based_ros2_control for Isaac Sim, ...) can be used with
 * simulation:=true.
 */
class ros2_control_crospi : public RobotDriver {
    public:
        typedef std::shared_ptr<ros2_control_crospi> SharedPtr;


    private:

        enum class CommandInterface { VELOCITY, POSITION };

        // ---------------- configuration (construct()) ----------------
        double periodicity_;
        std::vector<std::string> hardware_joints_;   // joint names in the <ros2_control> tag, in the order of robot_joints
        std::string robot_description_path_;
        std::vector<std::pair<std::string, std::string>> xacro_arguments_;
        std::vector<std::string> hardware_components_;   // empty: all components of the robot description
        CommandInterface command_interface_;
        double initialization_timeout_;
        double initialization_settle_time_;
        std::string ros_node_name_;
        std::string force_torque_sensor_name_;   // empty: not used
        std::string pose_sensor_name_;           // empty: not used

        // ---------------- ros2_control ----------------
        rclcpp::Node::SharedPtr node_;
        rclcpp::Executor::SharedPtr executor_;
        std::thread executor_thread_;
        std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;
        std::vector<std::string> started_command_interfaces_;
        std::vector<hardware_interface::LoanedCommandInterface> joint_commands_;
        std::vector<hardware_interface::LoanedStateInterface> joint_position_states_;
        std::vector<hardware_interface::LoanedStateInterface> joint_velocity_states_;   // empty if not reported
        std::vector<hardware_interface::LoanedStateInterface> joint_effort_states_;     // empty if not reported
        std::vector<hardware_interface::LoanedStateInterface> wrench_states_;           // force.xyz, torque.xyz
        std::vector<hardware_interface::LoanedStateInterface> pose_states_;             // position.xyz, orientation.xyzw
        rclcpp::Duration period_{0, 0};

        // ---------------- data exchanged with crospi ----------------
        robotdrivers::DynamicJointDataField setpoint_joint_vel_;
        robotdrivers::DynamicJointDataField joint_pos_;
        robotdrivers::DynamicJointDataField joint_vel_;
        robotdrivers::DynamicJointDataField joint_torque_;
        robotdrivers::Vector3Field cartesian_pos_;
        robotdrivers::QuaternionField cartesian_quat_;
        robotdrivers::ScrewField cartesian_wrench_;
        std::vector<double> position_command_;   // integrated position command (command_interface = position)

        // update() runs in the real-time thread of crospi, while finalize() is called both from that thread and from
        // the main thread (crospi safe_shutdown()). update() only try-locks this mutex, so it never blocks.
        std::mutex hardware_mutex_;
        bool initialized_ = false;
        bool finalized_ = false;
        bool error_reported_ = false;

        void load_hardware(const std::string& robot_description);
        void activate_hardware();
        void claim_interfaces();
        bool wait_for_valid_state();
        void read_states();
        void report_error(const std::string& message, volatile std::atomic<bool>& stopFlag);
        void release_hardware();


    public:
        ros2_control_crospi();

        virtual void construct(std::string robot_name,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

        /**
         * Loads the robot description and the hardware components, activates them, claims the joint interfaces and
         * waits until valid joint positions are received. Returns false if any of these steps fails.
        */
        virtual bool initialize() override;

        virtual void update(volatile std::atomic<bool>& stopFlag) override;

        virtual void on_configure() override;

        virtual void on_activate() override;

        virtual void on_deactivate() override;

        virtual void on_cleanup() override;

        virtual void finalize() override;

        virtual ~ros2_control_crospi();
};

} // namespace etasl
