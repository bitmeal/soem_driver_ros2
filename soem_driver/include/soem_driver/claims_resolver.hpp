#ifndef SOEM_DRIVER__CLAIMS_RESOLVER_HPP_
#define SOEM_DRIVER__CLAIMS_RESOLVER_HPP_

#include <string>
#include <string_view>
#include <vector>
#include <map>
#include <utility>
#include <functional>
#include <algorithm>
#include <iterator>
#include <ranges>
#include <memory>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"

#include "soem_driver_slave_interface/soem_driver_slave.hpp"

namespace soem_driver
{
    using std::operator""sv;

    class AliasInterfaceFactory : public hardware_interface::ReadOnlyHandle
    {
    public:
        AliasInterfaceFactory() = delete;
        AliasInterfaceFactory(const AliasInterfaceFactory &other) = delete;
        AliasInterfaceFactory(AliasInterfaceFactory &&other) = default;

        double get_value() = delete;

        AliasInterfaceFactory(const hardware_interface::ReadOnlyHandle &target) : hardware_interface::ReadOnlyHandle(target){};

        template <typename T, std::enable_if_t<std::is_base_of_v<ReadOnlyHandle, T>, bool> = true>
        T makeInterface(const std::string &prefix, const std::string &name)
        {
            // return std::move(T{prefix, name, value_ptr_});
            return T{prefix, name, value_ptr_};
        }
    };

    class ECClaimsResolver
    {
    private:
        // map of slave names to all their interfaces
        std::unordered_map<std::string, std::pair<
                                            std::vector<hardware_interface::StateInterface>,
                                            std::vector<hardware_interface::CommandInterface>>>
            slave_interfaces;

        // map command interfaces from <joint> --> {<slave>, <interface>}; use in command mode switching
        std::unordered_map<std::string, std::pair<std::string, std::string>> joint_command_interface_map;

        // references to master structures
        std::unordered_map<std::string, std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave>> slaves;
        std::unordered_map<std::string, std::vector<std::string>> joint_claims;

    public:
        std::string __logger_name;
        ECClaimsResolver() : __logger_name("ECClaimsResolver"){};
        void init(
            const std::unordered_map<std::string, std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave>> &slaves,
            const std::unordered_map<std::string, std::vector<std::string>> &joint_claims)

        {
            // store data
            this->slaves = slaves;
            this->joint_claims = joint_claims;
            // import all slaves interfaces
            std::transform(this->slaves.begin(), this->slaves.end(), std::inserter(slave_interfaces, slave_interfaces.end()),
                           [](auto slave) -> std::decay_t<decltype(slave_interfaces)>::value_type
                           {
                               return std::decay_t<decltype(slave_interfaces)>::value_type({slave.first,
                                                                                            std::decay_t<decltype(slave_interfaces)>::mapped_type({slave.second->export_state_interfaces(),
                                                                                                                                                   slave.second->export_command_interfaces()})});
                           });
        };

        const std::vector<std::string_view> tokenize_string(std::string_view input, std::string_view delimiter, size_t min_size = 0, std::string default_val = "") const
        {
            std::vector<std::string_view> tokens{};

            auto tokenizer = std::views::split(input, delimiter);
            std::transform(tokenizer.begin(), tokenizer.end(), std::back_inserter(tokens), [](auto &&token)
                           { 
                            // allow compilation without string_view from ranges constructor (P1989R2 & P2210R2)
                            return std::string_view(&*token.begin(), std::ranges::distance(token)); });

            if (min_size && tokens.size() < min_size)
            {
                tokens.resize(min_size, default_val);
            }

            return tokens;
        };

        // allow structured binding to tokens with container of known size at compile time
        const std::array<std::string, 4> tokenize_interface(std::string_view input) const
        {
            std::array<std::string, 4> tokens_binder;
            auto tokens = tokenize_string(input, "/"sv, 4, "");
            std::copy(tokens.begin(), tokens.end(), tokens_binder.begin());

            return tokens_binder;
        }

        // interfaces are built when needed (command interfaces are moveable, but not copyable)
        std::vector<hardware_interface::StateInterface> export_state_interfaces()
        {
            std::vector<hardware_interface::StateInterface> state_interfaces;

            // TODO(bitmeal): rework using ranges with transform, filter and std::optional?

            // iterate over all joints
            std::for_each(joint_claims.begin(), joint_claims.end(), [&](auto &&joint_claim)
                          {
                    // keep track of mapped interfaces per joint
                    std::vector<std::string> mappend_interfaces{};

                    auto& [joint, claims] = joint_claim;
                    // for each claim of a joint, export the referenced interfaces
                    std::for_each(claims.begin(), claims.end(), [&](auto &&claim_id)
                    {
                        // tokenize claim to: slave, claim, type, name (convert to string from string_view for easier logging)
                        auto& [slave, claim, type, name] = tokenize_interface(claim_id);

                        // shall use state interfaces, as in: whole claim (state and command) or state only
                        if(type.empty() || !type.compare("state"))
                        {
                            // find slave
                            auto slave_if_it = slave_interfaces.find(slave);
                            if(slave_if_it != slave_interfaces.end())
                            {
                                // bind both types of interfaces for readability
                                auto& [slave_state_interfaces, slave_command_interfaces] = slave_if_it->second;

                                auto& interfaces = slave_state_interfaces;

                                typename std::remove_cvref_t<decltype(interfaces)>::iterator interfaces_begin;
                                typename std::remove_cvref_t<decltype(interfaces)>::iterator interfaces_end;
                                
                                // claim all interfaces
                                if(name.empty())
                                {
                                    interfaces_begin = interfaces.begin();
                                    interfaces_end = interfaces.end();
                                }
                                // claim by singular interface name
                                else
                                {
                                    interfaces_begin = std::find_if(interfaces.begin(), interfaces.end(), [&](auto &&interface){
                                        return (
                                            !claim.compare(interface.get_prefix_name()) &&
                                            !name.compare(interface.get_interface_name())
                                            );
                                    });

                                    interfaces_end = (interfaces_begin != interfaces.end() ? std::next(interfaces_begin) : interfaces.end());
                                }

                                // iterate over all interfaces and find matching claims and interfaces
                                std::for_each(interfaces_begin, interfaces_end, [&](auto&& interface){
                                    // check for correct claim
                                    if(!claim.compare(interface.get_prefix_name()))
                                    {
                                        // check if we already have an interface of this name for this joint
                                        if(std::find(mappend_interfaces.begin(), mappend_interfaces.end(), interface.get_interface_name()) == mappend_interfaces.end())
                                        {
                                            auto alias_factory = AliasInterfaceFactory{interface};
                                            mappend_interfaces.push_back(interface.get_interface_name());
                                            state_interfaces.emplace_back(
                                                    alias_factory.makeInterface<hardware_interface::StateInterface>(slave, interface.get_interface_name())
                                                );

                                            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "mapping state interface: %s/%s to %s/%s", joint.c_str(), interface.get_interface_name().c_str(), slave.c_str(), interface.get_name().c_str());
                                        }
                                        else
                                        {
                                            RCLCPP_WARN(rclcpp::get_logger(__logger_name), "joint interface %s/%s already satisfied, while trying to map slave state interface %s/%s", joint.c_str(), interface.get_interface_name().c_str(), slave.c_str(), interface.get_name().c_str());
                                        }
                                    }
                                });
                            }
                            else
                            {
                                RCLCPP_WARN(rclcpp::get_logger(__logger_name), "could not find slave %s to resolve claim %s", slave.c_str(), claim.c_str());
                                return;
                            }
                        }
                    }); });

            return state_interfaces;
        };

        std::vector<hardware_interface::CommandInterface> export_command_interfaces()
        {
            std::vector<hardware_interface::CommandInterface> command_interfaces;

            // TODO(bitmeal): rework using ranges with transform, filter and std::optional?

            // iterate over all joints
            std::for_each(joint_claims.begin(), joint_claims.end(), [&](auto &&joint_claim)
                          {
                    // keep track of mapped interfaces per joint
                    std::vector<std::string> mappend_interfaces{};

                    auto& [joint, claims] = joint_claim;
                    // for each claim of a joint, export the referenced interfaces
                    std::for_each(claims.begin(), claims.end(), [&](auto &&claim_id)
                    {
                        // tokenize claim to: slave, claim, type, name (convert to string from string_view for easier logging)
                        auto& [slave, claim, type, name] = tokenize_interface(claim_id);

                        // shall use command interfaces, as in: whole claim (state and command) or state only
                        if(type.empty() || !type.compare("command"))
                        {
                            // find slave
                            auto slave_if_it = slave_interfaces.find(slave);
                            if(slave_if_it != slave_interfaces.end())
                            {
                                // bind both types of interfaces for readability
                                auto& [slave_state_interfaces, slave_command_interfaces] = slave_if_it->second;

                                auto& interfaces = slave_command_interfaces;

                                typename std::remove_cvref_t<decltype(interfaces)>::iterator interfaces_begin;
                                typename std::remove_cvref_t<decltype(interfaces)>::iterator interfaces_end;
                                
                                // claim all interfaces
                                if(name.empty())
                                {
                                    interfaces_begin = interfaces.begin();
                                    interfaces_end = interfaces.end();
                                }
                                // claim by singular interface name
                                else
                                {
                                    interfaces_begin = std::find_if(interfaces.begin(), interfaces.end(), [&](auto &&interface){
                                        return (
                                            !claim.compare(interface.get_prefix_name()) &&
                                            !name.compare(interface.get_interface_name())
                                            );
                                    });

                                    interfaces_end = (interfaces_begin != interfaces.end() ? std::next(interfaces_begin) : interfaces.end());
                                }

                                // iterate over all interfaces and find matching claims and interfaces
                                std::for_each(interfaces_begin, interfaces_end, [&](auto&& interface){
                                    // check for correct claim
                                    if(!claim.compare(interface.get_prefix_name()))
                                    {
                                        // check if we already have an interface of this name for this joint
                                        if(std::find(mappend_interfaces.begin(), mappend_interfaces.end(), interface.get_interface_name()) == mappend_interfaces.end())
                                        {
                                            auto alias_factory = AliasInterfaceFactory{interface};
                                            mappend_interfaces.push_back(interface.get_interface_name());
                                            command_interfaces.emplace_back(
                                                        alias_factory.makeInterface<hardware_interface::CommandInterface>(slave, interface.get_interface_name())
                                                );

                                            // store mapping for resolution in command mode changes
                                            joint_command_interface_map[joint + "/" + interface.get_interface_name()] = {slave, interface.get_name()};

                                            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "mapping command interface: %s/%s to %s/%s", joint.c_str(), interface.get_interface_name().c_str(), slave.c_str(), interface.get_name().c_str());
                                        }
                                        else
                                        {
                                            RCLCPP_WARN(rclcpp::get_logger(__logger_name), "joint interface %s/%s already satisfied, while trying to map slave command interface %s/%s", joint.c_str(), interface.get_interface_name().c_str(), slave.c_str(), interface.get_name().c_str());
                                        }
                                    }
                                });
                            }
                            else
                            {
                                RCLCPP_WARN(rclcpp::get_logger(__logger_name), "could not find slave %s to resolve claim %s", slave.c_str(), claim.c_str());
                                return;
                            }
                        }
                    }); });

            return command_interfaces;
        };

        // map from: joint interface --> slaves scoped claims interface
        // hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;
        // hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

        ~ECClaimsResolver(){};
    };
} // namespace soem_driver

#endif // SOEM_DRIVER__CLAIMS_RESOLVER_HPP_