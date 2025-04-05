#include "faulhaber/faulhaber.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "chrono"

namespace faulhaber {
    std::vector<hardware_interface::StateInterface> Faulhaber::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_name_,
                hardware_interface::HW_IF_POSITION, 
                &pos_estimate_
            )
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_name_,
                hardware_interface::HW_IF_VELOCITY, 
                &vel_estimate_
            )
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_name_,
                hardware_interface::HW_IF_CURRENT, 
                &curr_estimate_
            )
        );
        return state_interfaces;
    }
      
      std::vector<hardware_interface::CommandInterface> Faulhaber::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint_name_,
                hardware_interface::HW_IF_VELOCITY,
                &vel_setpoint_
            )
        );
        return command_interfaces;
    }
      

    hardware_interface::CallbackReturn Faulhaber::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(logger_, "Error initializing ActuatorInterface");
        return CallbackReturn::ERROR;
    }

    serial_port_ = info_.hardware_parameters["serial_port"];
    gear_ratio_ = std::stoi(info_.hardware_parameters["gear_ratio"]);

    drive_ = std::make_shared<MCDrive>();
    joint_name_ = info.joints[0].name;
    return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Faulhaber::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        msg_handler_.Open(serial_port_.c_str(), SERIAL_BAUDRATE);
        drive_->SetNodeId(1);
        drive_->Connect2MsgHandler(&msg_handler_);
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Faulhaber::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    return CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn Faulhaber::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        while (drive_step_ != 8) {
            auto now = std::chrono::system_clock::now().time_since_epoch();
            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
            drive_->SetActTime(millis);
            msg_handler_.Update(millis);
            switch(drive_step_) {
                case 0:
                    // THERE IS A BUG IN THE LIBRARY DO NOT UPDATE DRIVE STATUS BEFORE CALLING DISABLE
                    // THE LIBRARY WILL NOT SEND THE DISABLE COMMAND WHICH IS REQUIRED TO ENABLE LATER
                    if((drive_->DisableDrive()) == eMCDone)
                    {
                        RCLCPP_INFO(logger_, "0: Disable drive completed");
                        drive_step_ = 1;
                    }
                    break;
                case 1:
                    //first get a copy of the drive status
                    if((drive_->UpdateDriveStatus()) == eMCDone)
                    {
                        RCLCPP_INFO(logger_, "1: Update drive status completed");
                        drive_step_ = 2;
                    }
                    break;
                case 2:
                    //enable next
                    if((drive_->EnableDrive()) == eMCDone)
                    {
                        RCLCPP_INFO(logger_, "2: Enable drive completed");
                        drive_step_ = 3;
                    }
                    break;
                case 3:
                    //first get a copy of the drive status
                    if((drive_->UpdateDriveStatus()) == eMCDone)
                    {
                        RCLCPP_INFO(logger_, "3: Update drive status completed");
                        drive_step_ = 4;
                    }
                    break;
                case 4:
                    //config homing
                    if((drive_->ConfigureHoming(DEFAULT_HOMING_METHOD)) == eMCDone)
                    {
                        RCLCPP_INFO(logger_, "4: Homing configuration completed");
                        drive_step_ = 5;
                    }
                    break;
                case 5:
                    //start homing
                    //configuration is to already be enabled
                    if((drive_->DoHoming(0)) == eMCDone)
                    {
                        RCLCPP_INFO(logger_, "5: Homing completed");
                        drive_step_ = 6;
                    }
                    break;
                case 6:
                    //start homing
                    //configuration is to already be enabled
                    if((drive_->SetOpMode(VELOCITY_OPERATION_MODE)) == eMCDone)
                    {
                        RCLCPP_INFO(logger_, "6: Operation mode is set");
                        drive_step_ = 7;
                    }
                    break;
                case 7:
                    //start homing
                    //configuration is to already be enabled
                    if((drive_->UpdateDriveStatus()) == eMCDone)
                    {
                        RCLCPP_INFO(logger_, "7: Update drive status completed");
                        drive_step_ = 8;
                    }
                    break;
        }
    }
    return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Faulhaber::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        while (drive_->StopDrive() != eMCDone) {
            auto now = std::chrono::system_clock::now().time_since_epoch();
            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
            drive_->SetActTime(millis);
            msg_handler_.Update(millis);
        }
        return CallbackReturn::SUCCESS;
    }

    // TODO: export all logic to a state machine function
    hardware_interface::return_type Faulhaber::read(const rclcpp::Time& /*time*/,const rclcpp::Duration& /*period*/) {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        drive_->SetActTime(millis);
        msg_handler_.Update(millis);
        while (drive_->UploadParamterList(telemetries_, FAULHABER_NUM_TELEMETRIES) != eMCDone) {
            auto now = std::chrono::system_clock::now().time_since_epoch();
            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
            drive_->SetActTime(millis);
            msg_handler_.Update(millis);
        }
        pos_estimate_ = telemetries_[0].value / (gear_ratio_ * ENCODER_COUNTS_PER_TURN);
        vel_estimate_ = telemetries_[1].value / gear_ratio_;
        curr_estimate_ = telemetries_[2].value;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Faulhaber::write(const rclcpp::Time& /*time*/,const rclcpp::Duration& /*period*/) {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        drive_->SetActTime(millis);
        msg_handler_.Update(millis);
        while (drive_->MoveAtSpeed(vel_setpoint_) != eMCDone) {
            auto now = std::chrono::system_clock::now().time_since_epoch();
            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
            drive_->SetActTime(millis);
            msg_handler_.Update(millis);
        }
        return hardware_interface::return_type::OK;
    }

}  // namespace faulhaber

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(faulhaber::Faulhaber, hardware_interface::ActuatorInterface)
