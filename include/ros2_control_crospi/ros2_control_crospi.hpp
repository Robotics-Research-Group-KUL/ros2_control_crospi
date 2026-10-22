#pragma once

#include <string>


#include "robot_interfacing_utils/robotdriver.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"


namespace etasl {

class ros2_control_crospi : public RobotDriver {
    public:
        typedef std::shared_ptr<ros2_control_crospi> SharedPtr;


    private:
        
        // FeedbackMsg* feedback_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // std::string name;; Defined in super class RobotDriver at header file robotdriver.hpp

        double periodicity_;
        robotdrivers::DynamicJointDataField setpoint_joint_vel_;
        robotdrivers::DynamicJointDataField joint_pos_;

        static constexpr size_t DOF = 6; //Change to suit the degrees of freedom (i.e. number of joints) of the robot
        

    public:
        ros2_control_crospi();

        virtual void construct(std::string robot_name, 
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

        /**
         * will only return true if it has received values for all the joints named in jnames.
        */
        virtual bool initialize() override;

        virtual void update(volatile std::atomic<bool>& stopFlag) override;

        virtual void on_configure() override;

        virtual void on_activate() override;

        virtual void on_deactivate() override;

        virtual void on_cleanup() override;

        virtual void finalize() override;

        // virtual const std::string& getName() const override;

        virtual ~ros2_control_crospi();
};

} // namespace etasl
