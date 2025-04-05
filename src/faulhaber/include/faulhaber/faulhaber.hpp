#pragma once

#include "hardware_interface/actuator_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "faulhaber/MCDrive.h"
#include "faulhaber/MsgHandler.h"

#define POSITION_ADDRESS (0x6064)
#define VELOCITY_ADDRESS (0x606c)
#define CURRENT_ADDRESS (0x6078)

#define DEFAULT_SUB_ADDRESS (0)
#define DEFAULT_TELEMETRY_VALUE (0)
#define DEFAULT_TELEMETRY_LENGTH (4)

#define FAULHABER_NUM_TELEMETRIES (3)
#define ENCODER_COUNTS_PER_TURN (4096)

#define SERIAL_BAUDRATE (115200)
#define DEFAULT_HOMING_METHOD (34)
#define VELOCITY_OPERATION_MODE (3)

namespace faulhaber {
    class Faulhaber : public hardware_interface::ActuatorInterface {
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    
    private:
        uint8_t drive_step_;
        std::shared_ptr<MCDrive> drive_;
        std::string serial_port_;
        MsgHandler msg_handler_;
        rclcpp::Logger logger_ = rclcpp::get_logger("faulhaber_logger");
        std::uint32_t gear_ratio_ = 1;
        double vel_setpoint_ = 0;
        double pos_estimate_ = 0;
        double vel_estimate_ = 0;
        double curr_estimate_ = 0;
        std::string joint_name_;
        MCDriveParameter telemetries_[FAULHABER_NUM_TELEMETRIES] = {
            {POSITION_ADDRESS, DEFAULT_SUB_ADDRESS, DEFAULT_TELEMETRY_VALUE, DEFAULT_TELEMETRY_LENGTH},
            {VELOCITY_ADDRESS, DEFAULT_SUB_ADDRESS, DEFAULT_TELEMETRY_VALUE, DEFAULT_TELEMETRY_LENGTH},
            {CURRENT_ADDRESS, DEFAULT_SUB_ADDRESS, DEFAULT_TELEMETRY_VALUE, DEFAULT_TELEMETRY_LENGTH}
        };
    
    };
};
