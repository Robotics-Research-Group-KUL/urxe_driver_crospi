#include "urXe_driver_crospi/urXe_driver_crospi.hpp"
#include <fmt/format.h>
#include <iostream>

// includ library for sleep
#include <chrono>
#include <thread>

namespace etasl {
using namespace std::chrono_literals;

urXe_driver_crospi::urXe_driver_crospi()
    : periodicity(0.0), q_acceleration(0.0), alpha(0.0), DOF(0), rtde_control(nullptr), rtde_receive(nullptr), rtde_frequency(0.0), ros_node_(nullptr), wrench_filter_initialized(false)
{
}

void urXe_driver_crospi::construct(std::string robot_name, 
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker
)
{

    ip_address = jsonchecker->asString(config, "ip_address");
    periodicity = jsonchecker->asDouble(config, "periodicity");
    q_acceleration = jsonchecker->asDouble(config, "q_acceleration");
    alpha = jsonchecker->asDouble(config, "alpha");

    DOF = 6;

    AvailableFeedback available_fb{};
    available_fb.joint_pos = true;
    available_fb.joint_vel = true;
    available_fb.cartesian_wrench= true;

    constructPorts(DOF, available_fb); //Constructs all shared pointers and initialize data structures. Call after assigning available_feedback booleans.

    name = robot_name; //defined in RobotDriver super class.

    filtered_wrench.resize(DOF, 0.0); //Initialize filtered_wrench zero
    tare_data.resize(DOF, 0.0); //Initialize tare_data zero

    wrench_filter_initialized = false;

    //Initialize vectors
    initial_joints.resize(DOF, 0.0);
    joint_vel.resize(DOF, 0.0);
    joint_pos.resize(DOF, 0.0);
    vel_setpoint.resize(DOF, 0.0);
    wrench.resize(DOF, 0.0);

    // Initialize structure data
    setpoint_joint_vel_struct.data.resize(DOF, 0.0); //resize and initialize setpoint joint velocities to zero
    joint_pos_struct.data.resize(DOF, 0.0);
    joint_vel_struct.data.resize(DOF, 0.0);
    cartesian_wrench_struct.linear.x = 0.0;
    cartesian_wrench_struct.linear.y = 0.0;
    cartesian_wrench_struct.linear.z = 0.0;
    cartesian_wrench_struct.angular.x = 0.0;
    cartesian_wrench_struct.angular.y = 0.0;
    cartesian_wrench_struct.angular.z = 0.0;

    std::cout << "Constructed object of urXe_driver_crospi class with name: " << name << std::endl;

}

bool urXe_driver_crospi::initialize()
{       
    uint16_t flags = ur_rtde::RTDEControlInterface::FLAG_VERBOSE | ur_rtde::RTDEControlInterface::FLAG_UPLOAD_SCRIPT;

    std::vector<std::string> variables = {  "timestamp", 
        "actual_q",
        "actual_qd",
        "actual_current",
        "actual_TCP_pose",
        "actual_TCP_speed",
        "actual_TCP_force",
        "ft_raw_wrench"};

    rtde_frequency = 1.0/periodicity;

    std::cout << "Creating RTDEControlInterface (ip=" << ip_address << ", freq=" << rtde_frequency << ")" << std::endl;
    try {
        rtde_control = new ur_rtde::RTDEControlInterface(ip_address, rtde_frequency, flags);
    } catch (const std::exception& e) {
        std::cerr << "Exception creating RTDEControlInterface: " << e.what() << std::endl;
        rtde_control = nullptr;
    } catch (...) {
        std::cerr << "Unknown exception creating RTDEControlInterface" << std::endl;
        rtde_control = nullptr;
    }

    std::cout << "Creating RTDEReceiveInterface" << std::endl;
    try {
        rtde_receive = new ur_rtde::RTDEReceiveInterface(ip_address, rtde_frequency, variables, true);
    } catch (const std::exception& e) {
        std::cerr << "Exception creating RTDEReceiveInterface: " << e.what() << std::endl;
        rtde_receive = nullptr;
    } catch (...) {
        std::cerr << "Unknown exception creating RTDEReceiveInterface" << std::endl;
        rtde_receive = nullptr;
    }

    if (!rtde_control) {
        std::cerr << "RTDE Control pointer is null after construction" << std::endl;
        if (rtde_receive) { delete rtde_receive; rtde_receive = nullptr; }
        return false;
    }
    if (!rtde_control->isConnected()) {
        std::cout << "Failed to connect RTDE Control to the robot" << std::endl;
        delete rtde_control; rtde_control = nullptr;
        if (rtde_receive) { delete rtde_receive; rtde_receive = nullptr; }
        return false;
    }
    std::cout << "RTDE Control connected to the robot" << std::endl;
    try { rtde_control->zeroFtSensor(); } catch (...) { }

    if (!rtde_receive) {
        std::cerr << "RTDE Receive pointer is null after construction" << std::endl;
        // keep control connected but return failure to avoid proceeding with an incomplete setup
        return false;
    }
    if (rtde_receive->isConnected()){
        joint_pos = rtde_receive->getActualQ();
        joint_vel = rtde_receive->getActualQd();
        wrench = rtde_receive->getFtRawWrench();

        if (joint_pos.size() != static_cast<size_t>(DOF) || joint_vel.size() != static_cast<size_t>(DOF) || wrench.size() != static_cast<size_t>(DOF)) {
            std::cerr << "RTDE initial data size mismatch: got sizes " << joint_pos.size() << ", " << joint_vel.size() << ", " << wrench.size() << " expected " << DOF << std::endl;
            return false;
        }

        std::copy(joint_pos.begin(), joint_pos.end(), joint_pos_struct.data.begin()); // joint_pos_struct.data = joint_pos;
        writeFeedbackJointPosition(joint_pos_struct);

        std::copy(joint_vel.begin(), joint_vel.end(), joint_vel_struct.data.begin()); // joint_vel_struct.data = joint_vel;
        writeFeedbackJointVelocity(joint_vel_struct);

        cartesian_wrench_struct.linear.x = wrench[0];
        cartesian_wrench_struct.linear.y = wrench[1];
        cartesian_wrench_struct.linear.z = wrench[2];
        cartesian_wrench_struct.angular.x = wrench[3];
        cartesian_wrench_struct.angular.y = wrench[4];
        cartesian_wrench_struct.angular.z = wrench[5];
        writeFeedbackCartesianWrench(cartesian_wrench_struct);
    }
    else{
        std::cout << "Failed to connect RTDE Receive to the robot" << std::endl;
        return false;
    }

    // Ensure rclcpp is initialized before creating nodes/executors. Some
    // environments (notably certain Docker containers) may load this class
    // before the global rclcpp init has been called which causes crashes.
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    ros_node_ = std::make_shared<rclcpp::Node>("urXe_node");

    tare_load_cell_srv_ = ros_node_->create_service<std_srvs::srv::Empty>(
        "urXe/tare_load_cell",
        std::bind(&urXe_driver_crospi::tare_load_cell, this, std::placeholders::_1, std::placeholders::_2));

    ros_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(ros_node_);
        exec.spin();
    });

    return true;
}


void urXe_driver_crospi::update(volatile std::atomic<bool>& stopFlag)
{
    // std::cout << "urXe_driver_crospi::update() called =======================" << std::endl;
    if (rtde_control && rtde_control->isConnected()){
        readSetpointJointVelocity(setpoint_joint_vel_struct);
        assert(joint_pos.size() == setpoint_joint_vel_struct.data.size());
        std::copy(setpoint_joint_vel_struct.data.begin(), setpoint_joint_vel_struct.data.end(), vel_setpoint.begin());
        rtde_control->speedJ(vel_setpoint, q_acceleration, periodicity);
    }
    else{
        std::cout << "RTDE Control for ur is not connected" << std::endl;
        stopFlag.store(true);
    }
    

    if (rtde_receive && rtde_receive->isConnected()){
        joint_pos = rtde_receive->getActualQ();
        joint_vel = rtde_receive->getActualQd();
        wrench = rtde_receive->getFtRawWrench();
    }
    else{
        std::cout << "RTDE Receive for ur is not connected" << std::endl;
        stopFlag.store(true);
    }

    std::copy(joint_pos.begin(), joint_pos.end(), joint_pos_struct.data.begin());
    writeFeedbackJointPosition(joint_pos_struct);
    
    std::copy(joint_vel.begin(), joint_vel.end(), joint_vel_struct.data.begin());
    writeFeedbackJointVelocity(joint_vel_struct);

    cartesian_wrench_struct.linear.x = wrench[0];
    cartesian_wrench_struct.linear.y = wrench[1];
    cartesian_wrench_struct.linear.z = wrench[2];
    cartesian_wrench_struct.angular.x = wrench[3];
    cartesian_wrench_struct.angular.y = wrench[4];
    cartesian_wrench_struct.angular.z = wrench[5];

    // std::cout << "Raw Wrench: ";
    // for (const auto& val : wrench) {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;

    tare_mtx.lock();

    if (!wrench_filter_initialized) {
        filtered_wrench = wrench;
        wrench_filter_initialized = true;
    } else {
        for (size_t i = 0; i < 6; ++i) {
            filtered_wrench[i] = alpha * wrench[i] + (1.0 - alpha) * filtered_wrench[i];
        }
    }

    cartesian_wrench_struct.linear.x = filtered_wrench[0] - tare_data[0];
    cartesian_wrench_struct.linear.y = filtered_wrench[1] - tare_data[1];
    cartesian_wrench_struct.linear.z = filtered_wrench[2] - tare_data[2];
    cartesian_wrench_struct.angular.x = filtered_wrench[3] - tare_data[3];
    cartesian_wrench_struct.angular.y = filtered_wrench[4] - tare_data[4];
    cartesian_wrench_struct.angular.z = filtered_wrench[5] - tare_data[5];

    tare_mtx.unlock();

    writeFeedbackCartesianWrench(cartesian_wrench_struct);

}

void urXe_driver_crospi::on_configure() {
    // std::cout << "entering on configure =======================" << std::endl;

}

void urXe_driver_crospi::on_activate() 
{


}

void urXe_driver_crospi::on_deactivate() {
    // std::cout << "entering on deactivate =======================" << std::endl;

}

void urXe_driver_crospi::on_cleanup() {
    // std::cout << "entering on cleanup =======================" << std::endl;

}


void urXe_driver_crospi::finalize() {
    std::cout << "finalize() called =======================" << std::endl;
    joint_vel.resize(6, 0.0);
    if (rtde_control) {
        try { rtde_control->speedJ(joint_vel, q_acceleration, periodicity); } catch (...) {}
        try { rtde_control->stopJ(q_acceleration, false); } catch (...) {}
    }
    // disconnect
    if (rtde_control) {
        try {
            rtde_control->disconnect();
        } catch (...) {}
        delete rtde_control;
        rtde_control = nullptr;
    }
    if (rtde_receive) {
        try {
            rtde_receive->disconnect();
        } catch (...) {}
        delete rtde_receive;
        rtde_receive = nullptr;
    }

    // Shutdown ROS and join the thread to avoid races during destruction.
    if (ros_node_) {
        // Signal shutdown to executors/spinners. If rclcpp wasn't initialized
        // here we called init above, so shutdown is safe.
        try {
            rclcpp::shutdown();
        } catch (...) {}
    }
    if (ros_thread_.joinable()) {
        ros_thread_.join();
    }
    ros_node_.reset();
}

void urXe_driver_crospi::tare_load_cell(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                             std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    // TODO: Check rtde_control->zeroFtSensor();
    std::vector<double> average(6, 0.0);  // To store cumulative sums
    int n_data = 50;
    for (size_t i = 0; i < n_data; ++i)
    {
        tare_mtx.lock();
        for (size_t j = 0; j < 6; ++j) {
            average[j] += filtered_wrench[j];
        }
        tare_mtx.unlock();
        std::this_thread::sleep_for(periodicity * 1s);
    }
    // Final average
    for (size_t i = 0; i < 6; ++i) {
        average[i] /= static_cast<double>(n_data);
    }

    tare_mtx.lock();
    tare_data = average;
    filtered_wrench.resize(6, 0.0);
    wrench_filter_initialized = false;

    // Output result
    std::cout << "Average values:\n";
    for (double val : tare_data) {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    tare_mtx.unlock();
    
}

urXe_driver_crospi::~urXe_driver_crospi() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::urXe_driver_crospi, etasl::RobotDriver)
