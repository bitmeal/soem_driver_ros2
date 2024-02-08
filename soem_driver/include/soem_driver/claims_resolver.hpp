#ifndef SOEM_DRIVER__CLAIMS_RESOLVER_HPP_
#define SOEM_DRIVER__CLAIMS_RESOLVER_HPP_

#include <string>
#include <string_view>
#include <vector>
#include <map>
#include <utility>
// #include <functional>
// #include <algorithm>
// #include <iterator>
// #include <ranges>
#include <memory>
// #include <type_traits>

// #include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"

#include "soem_driver_slave_interface/soem_driver_slave.hpp"

namespace soem_driver
{
    using std::operator""sv;

    class ECClaimsResolver
    {
    private:
        // map of slave names to all their interfaces
        std::unordered_map<std::string, std::pair<
                                            std::vector<hardware_interface::StateInterface>,
                                            std::vector<hardware_interface::CommandInterface>>>
            slave_interfaces;

        // map command interfaces from <joint> --> {<slave>, <interface>}; use in command mode switching
        std::unordered_map<std::string, std::pair<std::string, std::string>> command_interface_map;

        // references to master structures
        std::unordered_map<std::string, std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave>> slaves;
        std::unordered_map<std::string, std::vector<std::string>> joint_claims;

        static const std::vector<std::string_view> tokenize_string(std::string_view input, std::string_view delimiter, size_t min_size = 0, std::string default_val = "");

        // allow structured binding to tokens with container of known size at compile time
        static const std::array<std::string, 4> tokenize_interface(std::string_view input);

        std::vector<hardware_interface::StateInterface>
        resolve_state_interfaces_by_joint(const std::string joint);

        std::pair<
            std::vector<hardware_interface::CommandInterface>,
            std::unordered_map<std::string, std::pair<std::string, std::string>>>
        resolve_command_interfaces_by_joint(const std::string joint);


    public:
        std::string __logger_name;
        ECClaimsResolver() : __logger_name("ECClaimsResolver"){};
        void init(
            const std::unordered_map<std::string, std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave>> &slaves,
            const std::unordered_map<std::string, std::vector<std::string>> &joint_claims);

        // std::vector<hardware_interface::StateInterface> export_state_interfaces();
        // std::vector<hardware_interface::CommandInterface> export_command_interfaces();

        std::unordered_map<std::string, std::vector<hardware_interface::StateInterface>> export_state_interfaces_by_joint();
        std::unordered_map<std::string, std::vector<hardware_interface::CommandInterface>> export_command_interfaces_by_joint();

        // map from: joint interface --> slaves scoped claims interface
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces);

        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces);

        ~ECClaimsResolver(){};
    };
} // namespace soem_driver

#endif // SOEM_DRIVER__CLAIMS_RESOLVER_HPP_