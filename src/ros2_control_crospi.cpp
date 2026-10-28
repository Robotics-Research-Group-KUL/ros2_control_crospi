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

#include "ros2_control_crospi/ros2_control_crospi.hpp"
#include "ros2_control_crospi/robot_description_loader.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <type_traits>

#include <fmt/format.h>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "crospi_utils/string_interpolate.hpp"

#if __has_include(<hardware_interface/types/resource_manager_params.hpp>)
#include <hardware_interface/types/resource_manager_params.hpp>
#define ROS2_CONTROL_CROSPI_HAS_RESOURCE_MANAGER_PARAMS 1
#endif

#if __has_include(<hardware_interface/introspection.hpp>)
#include <hardware_interface/introspection.hpp>
#define ROS2_CONTROL_CROSPI_HAS_INTROSPECTION 1
#endif

namespace etasl {

namespace {

const char* PLUGIN_NAME = "ros2_control_crospi";

// The accessors of the loaned interfaces changed across ROS 2 distributions (double get_value() / void set_value() in
// older releases, std::optional<T> get_optional<T>() / bool set_value() in newer ones). These helpers support both.
template <typename T, typename = void>
struct has_get_optional : std::false_type {};

template <typename T>
struct has_get_optional<T, std::void_t<decltype(std::declval<const T&>().template get_optional<double>())>>
    : std::true_type {};

template <typename Loaned>
std::optional<double> get_value(const Loaned& interface) {
    if constexpr (has_get_optional<Loaned>::value) {
        return interface.template get_optional<double>();
    } else {
        return interface.get_value();
    }
}

template <typename Loaned>
bool set_value(Loaned& interface, double value) {
    if constexpr (std::is_same_v<decltype(interface.set_value(value)), bool>) {
        return interface.set_value(value);
    } else {
        interface.set_value(value);
        return true;
    }
}

/** Copies a finite value of the interface into target; keeps the previous value otherwise. */
template <typename Loaned, typename T>
void copy_value(const Loaned& interface, T& target) {
    const std::optional<double> value = get_value(interface);
    if (value && std::isfinite(*value)) {
        target = static_cast<T>(*value);
    }
}

std::string join(const std::vector<std::string>& items) {
    std::string result;
    for (const auto& item : items) {
        result += (result.empty() ? "" : ", ") + item;
    }
    return result;
}

std::vector<std::string> read_string_array(const Json::Value& config, const std::string& key,
                                           std::shared_ptr<etasl::JsonChecker> jsonchecker) {
    std::vector<std::string> result;
    for (const auto& item : jsonchecker->asArray(config, key)) {
        result.push_back(item.asString());
    }
    return result;
}

} // namespace


ros2_control_crospi::ros2_control_crospi()
{
}

void ros2_control_crospi::construct(std::string robot_name,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    name = robot_name; //defined in RobotDriver super class.

    periodicity_ = jsonchecker->asDouble(config, "periodicity");
    if (periodicity_ <= 0.0) {
        throw std::runtime_error(fmt::format("[{}] periodicity must be positive.", PLUGIN_NAME));
    }
    period_ = rclcpp::Duration::from_seconds(periodicity_);

    const std::vector<std::string> robot_joints = read_string_array(config, "robot_joints", jsonchecker);
    hardware_joints_ = robot_joints;
    if (jsonchecker->is_member(config, "hardware_joints")) {
        hardware_joints_ = read_string_array(config, "hardware_joints", jsonchecker);
        if (hardware_joints_.size() != robot_joints.size()) {
            throw std::runtime_error(fmt::format("[{}] hardware_joints has {} elements, but robot_joints has {}.",
                                                 PLUGIN_NAME, hardware_joints_.size(), robot_joints.size()));
        }
    }

    // $[package] / package:// in the path and in the xacro arguments are resolved in initialize(), so that a missing
    // package (e.g. the hardware package of a real robot) is reported as an initialization failure.
    robot_description_path_ = jsonchecker->asString(config, "robot_description_path");

    xacro_arguments_.clear();
    if (jsonchecker->is_member(config, "xacro_arguments")) {
        const Json::Value& arguments = config["xacro_arguments"];
        if (!arguments.isObject()) {
            throw std::runtime_error(fmt::format("[{}] xacro_arguments must be an object.", PLUGIN_NAME));
        }
        for (const auto& key : arguments.getMemberNames()) {
            const Json::Value& value = arguments[key];
            if (!value.isString() && !value.isNumeric() && !value.isBool()) {
                throw std::runtime_error(fmt::format("[{}] xacro argument '{}' must be a string, number or boolean.",
                                                     PLUGIN_NAME, key));
            }
            xacro_arguments_.emplace_back(key, value.asString());
        }
    }

    const std::string command_interface = jsonchecker->asString(config, "command_interface");
    if (command_interface == "velocity") {
        command_interface_ = CommandInterface::VELOCITY;
    } else if (command_interface == "position") {
        command_interface_ = CommandInterface::POSITION;
    } else {
        throw std::runtime_error(fmt::format("[{}] command_interface must be 'velocity' or 'position' (got '{}').",
                                             PLUGIN_NAME, command_interface));
    }

    hardware_components_.clear();
    if (jsonchecker->is_member(config, "hardware_components")) {
        hardware_components_ = read_string_array(config, "hardware_components", jsonchecker);
    }
    initialization_timeout_ = jsonchecker->is_member(config, "initialization_timeout")
                                  ? jsonchecker->asDouble(config, "initialization_timeout") : 5.0;
    initialization_settle_time_ = jsonchecker->is_member(config, "initialization_settle_time")
                                      ? jsonchecker->asDouble(config, "initialization_settle_time") : 0.5;
    if (initialization_settle_time_ >= initialization_timeout_) {
        throw std::runtime_error(fmt::format("[{}] initialization_settle_time ({} s) must be smaller than "
                                             "initialization_timeout ({} s).", PLUGIN_NAME,
                                             initialization_settle_time_, initialization_timeout_));
    }
    ros_node_name_ = jsonchecker->is_member(config, "ros_node_name")
                         ? jsonchecker->asString(config, "ros_node_name") : PLUGIN_NAME;
    force_torque_sensor_name_ = jsonchecker->is_member(config, "force_torque_sensor_name")
                                    ? jsonchecker->asString(config, "force_torque_sensor_name") : "";
    pose_sensor_name_ = jsonchecker->is_member(config, "pose_sensor_name")
                            ? jsonchecker->asString(config, "pose_sensor_name") : "";

    // Optional feedback is only reported if crospi requests it (name_expr_*); its availability in the hardware is
    // checked in initialize(), when the hardware components are loaded.
    AvailableFeedback available_fb{};
    available_fb.joint_pos = true;
    available_fb.joint_vel = jsonchecker->is_member(config, "name_expr_joint_vel");
    available_fb.joint_torque = jsonchecker->is_member(config, "name_expr_joint_torque");
    available_fb.cartesian_pos = jsonchecker->is_member(config, "name_expr_cartesian_pos");
    available_fb.cartesian_quat = jsonchecker->is_member(config, "name_expr_cartesian_quat");
    available_fb.cartesian_wrench = jsonchecker->is_member(config, "name_expr_cartesian_wrench");

    if ((available_fb.cartesian_pos || available_fb.cartesian_quat) && pose_sensor_name_.empty()) {
        throw std::runtime_error(fmt::format("[{}] name_expr_cartesian_pos/name_expr_cartesian_quat require "
                                             "pose_sensor_name.", PLUGIN_NAME));
    }
    if (available_fb.cartesian_wrench && force_torque_sensor_name_.empty()) {
        throw std::runtime_error(fmt::format("[{}] name_expr_cartesian_wrench requires force_torque_sensor_name.",
                                             PLUGIN_NAME));
    }

    const std::size_t n = hardware_joints_.size();
    setpoint_joint_vel_.data.assign(n, 0.0f);
    joint_pos_.data.assign(n, 0.0f);
    joint_vel_.data.assign(n, 0.0f);
    joint_torque_.data.assign(n, 0.0f);
    position_command_.assign(n, 0.0);

    constructPorts(n, available_fb); //Constructs all shared pointers and initialize data structures. Call after assigning available_feedback booleans.

    std::cout << fmt::format("Constructed ros2_control robot driver '{}': robot_description={}, command_interface={}, "
                             "hardware joints=[{}]", name, robot_description_path_, command_interface,
                             join(hardware_joints_))
              << std::endl;
}

bool ros2_control_crospi::initialize()
{
    std::lock_guard<std::mutex> lock(hardware_mutex_);

    try {
        auto resolve = [](const std::string& value, auto resolver) {
            try {
                return resolver(value);
            } catch (const std::exception& e) {
                throw std::runtime_error(fmt::format("could not resolve '{}' (is the package installed and the "
                                                     "workspace sourced?): {}", value, e.what()));
            }
        };
        const std::string path = resolve(robot_description_path_, ros2_control_crospi_utils::resolve_path);
        std::vector<std::pair<std::string, std::string>> xacro_arguments;
        for (const auto& [key, value] : xacro_arguments_) {
            // $[package] and ${ENV}
            xacro_arguments.emplace_back(key, resolve(value, [](const std::string& v) { return etasl::string_interpolate(v); }));
        }
        const std::string robot_description = ros2_control_crospi_utils::load_robot_description(path, xacro_arguments);
        load_hardware(robot_description);
        activate_hardware();
        claim_interfaces();
        if (!wait_for_valid_state()) {
            release_hardware();
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << fmt::format("[{}] Could not initialize the robot driver '{}': {}", PLUGIN_NAME, name, e.what())
                  << std::endl;
        release_hardware();
        return false;
    }

    // Initial commands: standstill, or the measured position in position mode.
    for (std::size_t i = 0; i < joint_commands_.size(); ++i) {
        position_command_[i] = joint_pos_.data[i];
        const double command = (command_interface_ == CommandInterface::VELOCITY) ? 0.0 : position_command_[i];
        if (!set_value(joint_commands_[i], command)) {
            std::cerr << fmt::format("[{}] Could not set the initial command of '{}'.", PLUGIN_NAME,
                                     joint_commands_[i].get_name()) << std::endl;
        }
    }
    resource_manager_->write(node_->now(), period_);

    writeFeedbackJointPosition(joint_pos_);
    if (isJointVelAvailable()) writeFeedbackJointVelocity(joint_vel_);
    if (isJointTorqueAvailable()) writeFeedbackJointTorque(joint_torque_);
    if (isCartesianPosAvailable()) writeFeedbackCartesianPosition(cartesian_pos_);
    if (isCartesianQuatAvailable()) writeFeedbackCartesianQuaternion(cartesian_quat_);
    if (isCartesianWrenchAvailable()) writeFeedbackCartesianWrench(cartesian_wrench_);

    std::string initial_joints;
    for (float q : joint_pos_.data) {
        initial_joints += fmt::format("{}{:.4f}", initial_joints.empty() ? "" : ", ", q);
    }
    std::cout << fmt::format("Initialized ros2_control robot driver '{}': command interfaces [{}], initial joints [{}]",
                             name, join(started_command_interfaces_), initial_joints)
              << std::endl;

    initialized_ = true;
    return true;
}

void ros2_control_crospi::load_hardware(const std::string& robot_description)
{
    node_ = std::make_shared<rclcpp::Node>(ros_node_name_);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    const unsigned int update_rate = static_cast<unsigned int>(std::lround(1.0 / periodicity_));

#ifdef ROS2_CONTROL_CROSPI_HAS_INTROSPECTION
    // The hardware components register their interfaces for introspection in this registry (normally created by the
    // controller_manager). Its data is not published.
    INITIALIZE_ROS2_CONTROL_INTROSPECTION_REGISTRY(node_, hardware_interface::DEFAULT_INTROSPECTION_TOPIC,
                                                   hardware_interface::DEFAULT_REGISTRY_KEY);
#endif

#ifdef ROS2_CONTROL_CROSPI_HAS_RESOURCE_MANAGER_PARAMS
    // The executor is given to the hardware components, so that components that use a ROS 2 node are spun as well.
    hardware_interface::ResourceManagerParams params;
    params.robot_description = robot_description;
    params.clock = node_->get_clock();
    params.logger = node_->get_logger();
    params.node_namespace = node_->get_namespace();
    params.executor = executor_;
    params.update_rate = update_rate;
    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(params, true);
#else
    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(
        robot_description, node_->get_node_clock_interface(), node_->get_node_logging_interface(), false, update_rate);
#endif

    if (!resource_manager_->are_components_initialized()) {
        throw std::runtime_error("the hardware components of the robot description could not be loaded and "
                                 "initialized (see the messages of the resource manager above).");
    }

    executor_thread_ = std::thread([this]() {
        try {
            executor_->spin();
        } catch (const std::exception& e) {
            std::cerr << fmt::format("[{}] The executor of node '{}' stopped: {}", PLUGIN_NAME, ros_node_name_, e.what())
                      << std::endl;
        }
    });
}

void ros2_control_crospi::activate_hardware()
{
    const auto& components_status = resource_manager_->get_components_status();
    std::vector<std::string> available;
    for (const auto& [component, info] : components_status) {
        available.push_back(component);
    }
    if (available.empty()) {
        throw std::runtime_error("the robot description does not contain any <ros2_control> tag.");
    }

    const std::vector<std::string> components = hardware_components_.empty() ? available : hardware_components_;
    rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                   hardware_interface::lifecycle_state_names::ACTIVE);
    for (const auto& component : components) {
        if (components_status.find(component) == components_status.end()) {
            throw std::runtime_error(fmt::format("the hardware component '{}' does not exist. Available hardware "
                                                 "components: [{}]", component, join(available)));
        }
        if (resource_manager_->set_component_state(component, active) != hardware_interface::return_type::OK) {
            throw std::runtime_error(fmt::format("the hardware component '{}' could not be activated.", component));
        }
    }
}

void ros2_control_crospi::claim_interfaces()
{
    auto claim_state = [this](const std::string& key) {
        if (!resource_manager_->state_interface_is_available(key)) {
            throw std::runtime_error(fmt::format("the state interface '{}' is not available. Available state "
                                                 "interfaces: [{}]", key,
                                                 join(resource_manager_->available_state_interfaces())));
        }
        return resource_manager_->claim_state_interface(key);
    };

    const std::string command_type = (command_interface_ == CommandInterface::VELOCITY)
                                         ? hardware_interface::HW_IF_VELOCITY : hardware_interface::HW_IF_POSITION;
    std::vector<std::string> command_keys;
    for (const auto& joint : hardware_joints_) {
        const std::string key = joint + "/" + command_type;
        if (!resource_manager_->command_interface_is_available(key)) {
            throw std::runtime_error(fmt::format("the command interface '{}' is not available. Available command "
                                                 "interfaces: [{}]", key,
                                                 join(resource_manager_->available_command_interfaces())));
        }
        command_keys.push_back(key);
    }

    // Lets the hardware switch its control mode (e.g. velocity mode of a UR) before the interfaces are claimed,
    // in the same way as the controller_manager does when a controller is activated.
    if (!resource_manager_->prepare_command_mode_switch(command_keys, {})) {
        throw std::runtime_error(fmt::format("the hardware rejected the command interfaces [{}].", join(command_keys)));
    }
    if (!resource_manager_->perform_command_mode_switch(command_keys, {})) {
        throw std::runtime_error(fmt::format("the hardware could not switch to the command interfaces [{}].",
                                             join(command_keys)));
    }
    started_command_interfaces_ = command_keys;

    joint_commands_.reserve(command_keys.size());
    for (const auto& key : command_keys) {
        joint_commands_.emplace_back(resource_manager_->claim_command_interface(key));
    }

    joint_position_states_.reserve(hardware_joints_.size());
    for (const auto& joint : hardware_joints_) {
        joint_position_states_.emplace_back(claim_state(joint + "/" + hardware_interface::HW_IF_POSITION));
    }
    if (isJointVelAvailable()) {
        for (const auto& joint : hardware_joints_) {
            joint_velocity_states_.emplace_back(claim_state(joint + "/" + hardware_interface::HW_IF_VELOCITY));
        }
    }
    if (isJointTorqueAvailable()) {
        for (const auto& joint : hardware_joints_) {
            joint_effort_states_.emplace_back(claim_state(joint + "/" + hardware_interface::HW_IF_EFFORT));
        }
    }
    if (isCartesianWrenchAvailable()) {
        for (const char* axis : {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"}) {
            wrench_states_.emplace_back(claim_state(force_torque_sensor_name_ + "/" + axis));
        }
    }
    if (isCartesianPosAvailable() || isCartesianQuatAvailable()) {
        for (const char* axis : {"position.x", "position.y", "position.z",
                                 "orientation.x", "orientation.y", "orientation.z", "orientation.w"}) {
            pose_states_.emplace_back(claim_state(pose_sensor_name_ + "/" + axis));
        }
    }
}

bool ros2_control_crospi::wait_for_valid_state()
{
    // Some hardware reports placeholder values (e.g. zeros for a UR) until its first packets arrive, so the hardware is
    // read for at least initialization_settle_time before the initial state is taken.
    const auto start = std::chrono::steady_clock::now();
    const auto settled = start + std::chrono::duration<double>(initialization_settle_time_);
    const auto deadline = start + std::chrono::duration<double>(initialization_timeout_);
    while (true) {
        const auto status = resource_manager_->read(node_->now(), period_);
        if (status.result != hardware_interface::return_type::OK) {
            std::cerr << fmt::format("[{}] read() of the hardware components [{}] failed during initialization.",
                                     PLUGIN_NAME, join(status.failed_hardware_names)) << std::endl;
            return false;
        }

        std::vector<std::string> invalid;
        for (const auto& state : joint_position_states_) {
            const std::optional<double> value = get_value(state);
            if (!value || !std::isfinite(*value)) {
                invalid.push_back(state.get_name());
            }
        }
        if (invalid.empty() && std::chrono::steady_clock::now() >= settled) {
            read_states();
            if (std::all_of(joint_pos_.data.begin(), joint_pos_.data.end(), [](float q) { return q == 0.0f; })) {
                std::cerr << fmt::format("[{}] WARNING: all the initial joint positions are exactly zero. If this is "
                                         "not the actual configuration of the robot, the hardware has not sent its "
                                         "state yet: increase initialization_settle_time.", PLUGIN_NAME) << std::endl;
            }
            return true;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            std::cerr << fmt::format("[{}] No valid joint positions received within {} s for [{}].", PLUGIN_NAME,
                                     initialization_timeout_, join(invalid)) << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(periodicity_));
    }
}

void ros2_control_crospi::read_states()
{
    for (std::size_t i = 0; i < joint_position_states_.size(); ++i) {
        copy_value(joint_position_states_[i], joint_pos_.data[i]);
    }
    for (std::size_t i = 0; i < joint_velocity_states_.size(); ++i) {
        copy_value(joint_velocity_states_[i], joint_vel_.data[i]);
    }
    for (std::size_t i = 0; i < joint_effort_states_.size(); ++i) {
        copy_value(joint_effort_states_[i], joint_torque_.data[i]);
    }
    if (!wrench_states_.empty()) {
        copy_value(wrench_states_[0], cartesian_wrench_.linear.x);
        copy_value(wrench_states_[1], cartesian_wrench_.linear.y);
        copy_value(wrench_states_[2], cartesian_wrench_.linear.z);
        copy_value(wrench_states_[3], cartesian_wrench_.angular.x);
        copy_value(wrench_states_[4], cartesian_wrench_.angular.y);
        copy_value(wrench_states_[5], cartesian_wrench_.angular.z);
    }
    if (!pose_states_.empty()) {
        copy_value(pose_states_[0], cartesian_pos_.x);
        copy_value(pose_states_[1], cartesian_pos_.y);
        copy_value(pose_states_[2], cartesian_pos_.z);
        copy_value(pose_states_[3], cartesian_quat_.qx);
        copy_value(pose_states_[4], cartesian_quat_.qy);
        copy_value(pose_states_[5], cartesian_quat_.qz);
        copy_value(pose_states_[6], cartesian_quat_.qw);
    }
}

void ros2_control_crospi::report_error(const std::string& message, volatile std::atomic<bool>& stopFlag)
{
    if (!error_reported_) {
        std::cerr << fmt::format("[{}] Robot driver '{}': {} Stopping the control loop.", PLUGIN_NAME, name, message)
                  << std::endl;
        error_reported_ = true;
    }
    stopFlag.store(true);
}


void ros2_control_crospi::update(volatile std::atomic<bool>& stopFlag)
{
    std::unique_lock<std::mutex> lock(hardware_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || finalized_) {
        return; // finalize() is running or has already released the hardware
    }
    if (!initialized_) {
        report_error("the hardware was not initialized successfully.", stopFlag);
        return;
    }

    const rclcpp::Time time = node_->now();

    const auto read_status = resource_manager_->read(time, period_);
    if (read_status.result != hardware_interface::return_type::OK) {
        report_error(fmt::format("read() of the hardware components [{}] failed.",
                                 join(read_status.failed_hardware_names)), stopFlag);
        return;
    }
    read_states();

    writeFeedbackJointPosition(joint_pos_);
    if (isJointVelAvailable()) writeFeedbackJointVelocity(joint_vel_);
    if (isJointTorqueAvailable()) writeFeedbackJointTorque(joint_torque_);
    if (isCartesianPosAvailable()) writeFeedbackCartesianPosition(cartesian_pos_);
    if (isCartesianQuatAvailable()) writeFeedbackCartesianQuaternion(cartesian_quat_);
    if (isCartesianWrenchAvailable()) writeFeedbackCartesianWrench(cartesian_wrench_);

    readSetpointJointVelocity(setpoint_joint_vel_);

    for (std::size_t i = 0; i < joint_commands_.size(); ++i) {
        const double velocity = setpoint_joint_vel_.data[i];
        if (command_interface_ == CommandInterface::VELOCITY) {
            (void) set_value(joint_commands_[i], velocity);
        } else {
            position_command_[i] += velocity * periodicity_;
            (void) set_value(joint_commands_[i], position_command_[i]);
        }
    }

    const auto write_status = resource_manager_->write(time, period_);
    if (write_status.result != hardware_interface::return_type::OK) {
        report_error(fmt::format("write() of the hardware components [{}] failed.",
                                 join(write_status.failed_hardware_names)), stopFlag);
    }
}

void ros2_control_crospi::on_configure() {

}

void ros2_control_crospi::on_activate()
{

}

void ros2_control_crospi::on_deactivate() {

}

void ros2_control_crospi::on_cleanup() {

}

void ros2_control_crospi::release_hardware()
{
    if (resource_manager_) {
        joint_commands_.clear();
        joint_position_states_.clear();
        joint_velocity_states_.clear();
        joint_effort_states_.clear();
        wrench_states_.clear();
        pose_states_.clear();

        if (!started_command_interfaces_.empty() &&
            resource_manager_->prepare_command_mode_switch({}, started_command_interfaces_)) {
            (void) resource_manager_->perform_command_mode_switch({}, started_command_interfaces_);
        }
        started_command_interfaces_.clear();

        // Deactivates and shuts down the hardware components that were configured (unconfigured components are
        // destroyed together with the resource manager).
        std::vector<std::pair<std::string, uint8_t>> components;
        for (const auto& [component, info] : resource_manager_->get_components_status()) {
            components.emplace_back(component, info.state.id());
        }
        rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                         hardware_interface::lifecycle_state_names::INACTIVE);
        rclcpp_lifecycle::State finalized(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
                                          hardware_interface::lifecycle_state_names::FINALIZED);
        for (const auto& [component, state] : components) {
            if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                resource_manager_->set_component_state(component, inactive);
            }
            if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
                state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                resource_manager_->set_component_state(component, finalized);
            }
        }
        resource_manager_.reset();
    }

    if (executor_) {
        executor_->cancel();
    }
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    executor_.reset();
    node_.reset();
    initialized_ = false;
}


void ros2_control_crospi::finalize() {
    std::lock_guard<std::mutex> lock(hardware_mutex_);
    if (finalized_) {
        return; // finalize() is called both by the thread of the driver and by crospi when it shuts down
    }
    finalized_ = true;

    if (initialized_) {
        // Last command before releasing the hardware: standstill (the last position command in position mode).
        for (std::size_t i = 0; i < joint_commands_.size(); ++i) {
            const double command = (command_interface_ == CommandInterface::VELOCITY) ? 0.0 : position_command_[i];
            (void) set_value(joint_commands_[i], command);
        }
        resource_manager_->write(node_->now(), period_);
    }
    release_hardware();

    std::cout << fmt::format("Finalized ros2_control robot driver '{}'.", name) << std::endl;
}



ros2_control_crospi::~ros2_control_crospi() {
    finalize();
};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::ros2_control_crospi, etasl::RobotDriver)
