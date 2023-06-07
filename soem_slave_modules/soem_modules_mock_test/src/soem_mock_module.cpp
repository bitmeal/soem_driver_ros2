#include "soem_slave_interface/soem_slave.hpp"
#include "rclcpp/rclcpp.hpp"

namespace soem_slave_modules
{
    class SOEMMockModule : public soem_slave_interface::SOEMSlave
    {
        bool init(std::unordered_map<std::string, std::string> parameters)
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module instantiated");

            return true;
        }

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces()
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module - call to: %s", __FUNCTION__);
            return {};
        };
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces()
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module - call to: %s", __FUNCTION__);
            return {};
        };

        virtual void configure(
            uint64_t vendor_id,
            uint64_t product_code,
            uint64_t revision_number,
            std::unordered_map<std::string, std::string> parameters)
        {
            RCLCPP_INFO(rclcpp::get_logger("soem_slave_modules/soem_mock_module"), "soem_mock_module - call to: %s", __FUNCTION__);
        };

        virtual void setup_SDO_hook(SDOwrite_t SDOwrite)
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
    soem_slave_modules::SOEMMockModule, soem_slave_interface::SOEMSlave)
