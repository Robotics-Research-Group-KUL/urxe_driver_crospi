#pragma once

#include <string>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>

#include "robot_interfacing_utils/robotdriver.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"

#include "std_srvs/srv/empty.hpp"
#include "rclcpp/rclcpp.hpp"

namespace etasl {

class urXe_driver_crospi : public RobotDriver {
    public:
        typedef std::shared_ptr<urXe_driver_crospi> SharedPtr;


    private:

        double periodicity, q_acceleration, alpha;

        std::vector<double> initial_joints;
        std::vector<double> joint_pos, joint_vel, wrench, filtered_wrench, vel_setpoint;

        robotdrivers::DynamicJointDataField setpoint_joint_vel_struct;
        robotdrivers::DynamicJointDataField joint_pos_struct;
        robotdrivers::DynamicJointDataField joint_vel_struct;
        robotdrivers::ScrewField cartesian_wrench_struct;

        std::mutex tare_mtx; // Mutex to protect the data
        int DOF;

        bool wrench_filter_initialized;
        
        std::string ip_address;

        std::string p_event_string;
        ur_rtde::RTDEControlInterface 	*rtde_control;
        ur_rtde::RTDEReceiveInterface 	*rtde_receive;

        double	                        rtde_frequency;

        std::shared_ptr<rclcpp::Node> ros_node_;
        std::thread ros_thread_;

        std::vector<double> tare_data;

        // ROS services
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr tare_load_cell_srv_;

        void tare_load_cell( 
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response);

    public:
        urXe_driver_crospi();

        virtual void construct(std::string robot_name,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

        virtual bool initialize() override;

        virtual void update(volatile std::atomic<bool>& stopFlag) override;

        virtual void on_configure() override;

        virtual void on_activate() override;

        virtual void on_deactivate() override;

        virtual void on_cleanup() override;

        virtual void finalize() override;

        virtual ~urXe_driver_crospi();
};

} // namespace etasl
