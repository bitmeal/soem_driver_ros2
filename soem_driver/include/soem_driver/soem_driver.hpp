
#ifndef SOEM_DRIVER__SOEM_DRIVER_HPP_
#define SOEM_DRIVER__SOEM_DRIVER_HPP_

#include <memory>
#include <vector>
#include <map>

#include <tinyxml2.h>
#include "hardware_interface/system_interface.hpp"
#include <pluginlib/class_loader.hpp>

#include "soem_slave_interface/soem_slave.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace soem_driver
{
    class SOEMDriver : public hardware_interface::SystemInterface
    {
    public:
        typedef struct EthercatSlaveInfo
        {
            std::string name;
            int alias;
            int position;
            std::string plugin_name;
            std::unordered_map<std::string, std::string> parameters;
            // std::vector<std::string> claims;
        } EthercatSlaveInfo;

        // typedef struct SlaveClaimInfo
        // {
        //     std::string slave_name;
        //     std::string claim_name;

        //     bool operator==(const SlaveClaimInfo &cmp) const
        //     {
        //         return (slave_name == cmp.slave_name && claim_name == cmp.slave_name);
        //     }
        // } SlaveClaimInfo;
        // typedef struct _SlaveClaimInfo_Hasher
        // {
        //     std::size_t operator()(const SlaveClaimInfo &c) const
        //     {
        //         return std::hash<std::string>()(c.slave_name + "/" + c.claim_name);
        //     }
        // } _SlaveClaimInfo_Hasher;

        // std::unordered_map<SlaveClaimInfo, bool, _SlaveClaimInfo_Hasher> claims_map;

        SOEMDriver();
        ~SOEMDriver();

        // CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        // methods ordered by expected order of execution during operation

        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

        // hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;
        // hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    private:
        std::string __logger_name;

        pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> slave_loader_{
            "soem_slave_interface", "soem_slave_interface::SOEMSlave"};


        std::vector<std::shared_ptr<soem_slave_interface::SOEMSlave>> slaves;
        std::vector<EthercatSlaveInfo> slave_infos;
        std::unordered_map<std::string, std::vector<std::string>> joint_claims;
        std::string ec_interface;

        std::string _get_text_for_element(
            const tinyxml2::XMLElement *element_it, const std::string &tag_name);
        std::string _get_attribute_value(
            const tinyxml2::XMLElement *element_it, const char *attribute_name, std::string tag_name);
        std::string _get_attribute_value(
            const tinyxml2::XMLElement *element_it, const char *attribute_name, const char *tag_name);
        std::unordered_map<std::string, std::string> _parse_parameters_from_xml(
            const tinyxml2::XMLElement *params_it);
        const tinyxml2::XMLElement *_parse_hardware_from_doc(tinyxml2::XMLDocument &doc, const std::string name);
        std::vector<EthercatSlaveInfo> _parse_slaves_from_hardware(const tinyxml2::XMLElement* hardware);
        EthercatSlaveInfo _parse_slave_info(const tinyxml2::XMLElement* slave);
        std::unordered_map<std::string, std::vector<std::string>> _parse_joint_claims(const hardware_interface::HardwareInfo &info);
        void parse_hardware_info(const hardware_interface::HardwareInfo &info);
    };
} // namespace soem_driver

#endif // SOEM_DRIVER__SOEM_DRIVER_HPP_
