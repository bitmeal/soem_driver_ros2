#include <cmath>

#include "soem_driver_slave_interface/soem_driver_slave.hpp"
#include "rclcpp/rclcpp.hpp"

namespace soem_slave_modules
{
    // TODO(bitmeal): add mock component parameters and state mirroring,
    // as in: https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html

    class SOEMMockModule : public soem_driver_slave_interface::SOEMDriverSlave
    {
        std::vector<double> state_interfaces;
        std::vector<double> command_interfaces;

        bool init(std::unordered_map<std::string, std::string> parameters)
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module instantiated");

            return true;
        }

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces()
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module - call to: %s", __FUNCTION__);

            state_interfaces.resize(6, NAN);
            return {
                {"joint_unit", "position", &state_interfaces[0]},
                {"joint_unit", "velocity", &state_interfaces[1]},
                {"joint_unit", "effort", &state_interfaces[2]},

                {"sensor_unit", "position", &state_interfaces[3]},
                {"sensor_unit", "velocity", &state_interfaces[4]},
                {"sensor_unit", "effort", &state_interfaces[5]},
            };
        };
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces()
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module - call to: %s", __FUNCTION__);

            command_interfaces.resize(6, NAN);
            return soem_driver::list_initialize_non_copyable_interface<hardware_interface::CommandInterface>(
                {{"joint_unit", "position", &command_interfaces[0]},
                 {"joint_unit", "velocity", &command_interfaces[1]},
                 {"joint_unit", "effort", &command_interfaces[2]},

                 {"actuator_unit", "position", &command_interfaces[3]},
                 {"actuator_unit", "velocity", &command_interfaces[4]},
                 {"actuator_unit", "effort", &command_interfaces[5]}});
        };

        virtual void configure(
            uint64_t vendor_id,
            uint64_t product_code,
            uint64_t revision_number,
            std::unordered_map<std::string, std::string> parameters)
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module - call to: %s", __FUNCTION__);
        };

        virtual void setup_SDO_hook(soem_driver::SDOwrite_t SDOwrite)
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module - call to: %s", __FUNCTION__);
        };

        virtual hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &)
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module - call to: %s", __FUNCTION__);
            return hardware_interface::return_type::OK;
        };
        virtual hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &)
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module - call to: %s", __FUNCTION__);
            return hardware_interface::return_type::OK;
        };
    };
} // namespace soem_slave_modules

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    soem_slave_modules::SOEMMockModule, soem_driver_slave_interface::SOEMDriverSlave)
