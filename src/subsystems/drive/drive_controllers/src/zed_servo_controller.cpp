// Copyright (c) 2025, UMDLoop
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "athena_drive_controllers/zed_servo_controller.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

//Included the Initial Position of pixhawk
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#define DEBUG_MODE 0 


namespace
{  // utility

using ControllerReferenceMsg = drive_controllers::ZedServoController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg)
{
    //Resets the input read value to 0
  msg->data = std::numeric_limits<int>::quiet_NaN();
}

}  // namespace
//drive_controllers? Or is it zed_servo_controller
namespace drive_controllers
{
    ZedServoController::ZedServoController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn ZedServoController::on_init()
    {
        control_mode_.initRT(control_mode_type::FAST);
        try
        {
            param_listener_ = std::make_shared<zed_servo_controller::ParamListener>(get_node());
        }
        catch (const std::exception & e)
        {
            fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }
    //Making my own ccommand_interface_configuration()
    //Read
    controller_interface::InterfaceConfiguration ZedServoController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        for (const auto & joint : joints)
        {
        command_interfaces_config.names.push_back(joint + "/" + "position");
        }
        //If you want to access single element
        //command_interfaces_config.names.push_back("zed_servo_joint/position");
        return command_interfaces_config;
    }
    //Write
    controller_interface::InterfaceConfiguration ZedServoController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto & joint : joints)
        {
            command_interfaces_config.names.push_back(joint + "/" + "position");
        }
        //command_interfaces_config.names.push_back("zed_servo_joint/position");
        return command_interfaces_config;
    }

    controller_interface::CallbackReturn ZedServoController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
    {
    // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
    // `on_activate` method in `JointTrajectoryController` for exemplary use of
    // `controller_interface::get_ordered_interfaces` helper function

    // Set default value in command
        reset_controller_reference_msg(*(input_ref_.readFromRT)());
        RCLCPP_INFO(get_node()->get_logger(),"Activated Controller Successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ZedServoController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
        // instead of a loop
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ZedServoController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        params_ = param_listener_->get_params();
        
        //Added Zed_Servo_Joint here
        //joints = params_.zed_servo_joint;
        //joints.push_back(params_.zed_servo_joint);
        joints.push_back(params_.zed_servo_joint);
        //joints.insert(joints.end(),params_.zed_servo_joint.begin(),params_.zed_servo_joint.end());

        // topics QoS
        auto subscribers_qos = rclcpp::SystemDefaultsQoS();
        subscribers_qos.keep_last(1);
        subscribers_qos.best_effort();

        // Reference Subscriber
        // ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
        //     "/gps_fix", subscribers_qos);
        

        std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
        reset_controller_reference_msg(msg);
        input_ref_.writeFromNonRT(msg);
        try
        {
            // State publisher
            s_publisher_ =
            get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
            state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
        }
        catch (const std::exception & e)
        {
            fprintf(
            stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
            e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        // TODO(anyone): Reserve memory in state publisher depending on the message type
        state_publisher_->lock();
        state_publisher_->msg_.header.frame_id = joints[0];
        state_publisher_->unlock();

        //RCLCPP_INFO(get_node()->get_logger(), "configure successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void ZedServoController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
    {
        input_ref_.writeFromNonRT(msg);
    }

    controller_interface::return_type ZedServoController::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
    {
        //Can update the current position + frame
        //RCLCPP_INFO(get_node()->get_logger(), "Position X %f, Y: %f Z: %f, Frame: %f\n", position_x, front_right_velocity, rear_left_velocity);
        auto current_ref = input_ref_.readFromRT();
        int linear_x;
        RCLCPP_INFO(get_node()->get_logger(),"Facing Angle: %i",linear_x);
        linear_x = (*current_ref)->data;
        

        return controller_interface::return_type::OK;
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_controllers::ZedServoController, controller_interface::ControllerInterface)
