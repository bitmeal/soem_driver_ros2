#include <functional>
#include <algorithm>
#include <iterator>
#include <ranges>
#include <memory>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"

#include "soem_driver/alias_interface_factory.hpp"

#include "soem_driver/claims_resolver.hpp"

namespace soem_driver
{

    void ECClaimsResolver::init(
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

    const std::vector<std::string_view> ECClaimsResolver::tokenize_string(std::string_view input, std::string_view delimiter, size_t min_size, std::string default_val)
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
    const std::array<std::string, 4> ECClaimsResolver::tokenize_interface(std::string_view input)
    {
        std::array<std::string, 4> tokens_binder;
        auto tokens = tokenize_string(input, "/"sv, 4, "");
        std::copy(tokens.begin(), tokens.end(), tokens_binder.begin());

        return tokens_binder;
    }

    std::vector<hardware_interface::StateInterface>
    ECClaimsResolver::resolve_state_interfaces_by_joint(const std::string joint)
    {
        std::vector<hardware_interface::StateInterface> joint_state_interfaces{};
        std::vector<std::string> joint_mappend_interfaces{};

        auto &claims = joint_claims[joint];

        std::for_each(claims.begin(), claims.end(), [&](auto &&claim_id)
                      {
                        // tokenize claim to: slave, claim, type, name (convert to string from string_view for easier logging)
                        auto&& [slave, claim, type, name] = tokenize_interface(claim_id);

                        // shall use state interfaces, as in: whole claim (state and command) or state only
                        if(type.empty() || !type.compare("state"))
                        {
                            // find slave
                            auto slave_if_it = slave_interfaces.find(slave);
                            if(slave_if_it != slave_interfaces.end())
                            {
                                auto&& [slave_state_interfaces, _] = slave_if_it->second;

                                auto&& interfaces = slave_state_interfaces;

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
                                        if(std::find(joint_mappend_interfaces.begin(), joint_mappend_interfaces.end(), interface.get_interface_name()) == joint_mappend_interfaces.end())
                                        {
                                            auto alias_factory = AliasInterfaceFactory{interface};
                                            joint_mappend_interfaces.push_back(interface.get_interface_name());
                                            joint_state_interfaces.emplace_back(std::move(
                                                    alias_factory.makeInterface<hardware_interface::StateInterface>(joint, interface.get_interface_name()))
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
                        } });

        return joint_state_interfaces;
    };

    // // interfaces are built when needed (command interfaces are moveable, but not copyable)
    // std::vector<hardware_interface::StateInterface> ECClaimsResolver::export_state_interfaces()
    // {
    //     std::vector<hardware_interface::StateInterface> state_interfaces;

    //     // TODO(bitmeal): rework using ranges with transform, filter and std::optional?

    //     // iterate over all joints
    //     std::for_each(joint_claims.begin(), joint_claims.end(), [&](auto &&single_joints_claims)
    //                   {
    //                       auto joint_state_interfaces = resolve_state_interfaces_by_joint(single_joints_claims.first);

    //                       // TODO(bitmeal): handle transmissions

    //                       // TODO(bitmeal): unify use of std::begin, std::end, .begin, .end
    //                       state_interfaces.reserve(state_interfaces.size() + joint_state_interfaces.size());
    //                       std::move(std::begin(joint_state_interfaces), std::end(joint_state_interfaces), std::back_inserter(state_interfaces));
    //                       // joint_command_interfaces.clear();
    //                   });

    //     return state_interfaces;
    // };

    // interfaces are built when needed (command interfaces are moveable, but not copyable)
    std::unordered_map<std::string, std::vector<hardware_interface::StateInterface>> ECClaimsResolver::export_state_interfaces_by_joint()
    {
        std::unordered_map<std::string, std::vector<hardware_interface::StateInterface>> state_interfaces;

        std::for_each(joint_claims.begin(), joint_claims.end(), [&](auto &&single_joints_claims)
                      {
                          auto&& joint_state_interfaces = resolve_state_interfaces_by_joint(single_joints_claims.first);
                          state_interfaces.emplace(std::make_pair(single_joints_claims.first, std::move(joint_state_interfaces))); });

        return state_interfaces;
    };

    std::pair<
        std::vector<hardware_interface::CommandInterface>,
        std::unordered_map<std::string, std::pair<std::string, std::string>>>
    ECClaimsResolver::resolve_command_interfaces_by_joint(const std::string joint)
    {
        std::vector<hardware_interface::CommandInterface> joint_command_interfaces{};
        std::unordered_map<std::string, std::pair<std::string, std::string>> joint_command_interface_map{};
        std::vector<std::string> joint_mappend_interfaces{};

        auto &claims = joint_claims[joint];

        // for each claim of a joint, export the referenced interfaces
        std::for_each(claims.begin(), claims.end(), [&](auto &&claim_id)
                      {
                        // tokenize claim to: slave, claim, type, name (convert to string from string_view for easier logging)
                        auto&& [slave, claim, type, name] = tokenize_interface(claim_id);

                        // shall use command interfaces, as in: whole claim (state and command) or command only
                        if(type.empty() || !type.compare("command"))
                        {
                            // find slave
                            auto slave_if_it = slave_interfaces.find(slave);
                            if(slave_if_it != slave_interfaces.end())
                            {
                                auto&& [_, slave_command_interfaces] = slave_if_it->second;

                                auto&& interfaces = slave_command_interfaces;

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
                                        if(std::find(joint_mappend_interfaces.begin(), joint_mappend_interfaces.end(), interface.get_interface_name()) == joint_mappend_interfaces.end())
                                        {
                                            auto alias_factory = AliasInterfaceFactory{interface};
                                            joint_mappend_interfaces.push_back(interface.get_interface_name());
                                            joint_command_interfaces.emplace_back(std::move(
                                                        alias_factory.makeInterface<hardware_interface::CommandInterface>(joint, interface.get_interface_name()))
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
                        } });

        return std::make_pair(
            std::move(joint_command_interfaces),
            std::move(joint_command_interface_map));
    };

    // std::vector<hardware_interface::CommandInterface> ECClaimsResolver::export_command_interfaces()
    // {
    //     std::vector<hardware_interface::CommandInterface> command_interfaces;

    //     // TODO(bitmeal): rework using ranges with transform, filter and std::optional?

    //     // iterate over all joints
    //     std::for_each(joint_claims.begin(), joint_claims.end(), [&](auto &&single_joints_claims)
    //                   {
    //                           auto [joint_command_interfaces, joint_command_interface_map] = resolve_command_interfaces_by_joint(single_joints_claims.first);

    //                           // TODO(bitmeal): handle transmissions

    //                           // TODO(bitmeal): unify use of std::begin, std::end, .begin, .end
    //                           command_interfaces.reserve(command_interfaces.size() + joint_command_interfaces.size());
    //                           std::move(std::begin(joint_command_interfaces), std::end(joint_command_interfaces), std::back_inserter(command_interfaces));
    //                           // joint_command_interfaces.clear();

    //                           command_interface_map.insert(std::begin(joint_command_interface_map), std::end(joint_command_interface_map)); });

    //     return command_interfaces;
    // };

    std::unordered_map<std::string, std::vector<hardware_interface::CommandInterface>> ECClaimsResolver::export_command_interfaces_by_joint()
    {
        std::unordered_map<std::string, std::vector<hardware_interface::CommandInterface>> command_interfaces;

        std::for_each(joint_claims.begin(), joint_claims.end(), [&](auto &&single_joints_claims)
                      {
                          auto&& [joint_command_interfaces, joint_command_interface_map] = resolve_command_interfaces_by_joint(single_joints_claims.first);

                          command_interface_map.insert(std::begin(joint_command_interface_map), std::end(joint_command_interface_map));
                          command_interfaces.emplace(std::make_pair(single_joints_claims.first, std::move(joint_command_interfaces))); });

        return command_interfaces;
    };

    // map from: joint interface --> slaves scoped claims interface
    hardware_interface::return_type ECClaimsResolver::prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces)
    {
        std::unordered_map<std::string, std::vector<std::string>> start_claims_by_slave;
        std::unordered_map<std::string, std::vector<std::string>> stop_claims_by_slave;

        std::for_each(start_interfaces.begin(), start_interfaces.end(), [&](auto &start_interface)
                      {
                auto [slave, claim] = command_interface_map[start_interface];
                start_claims_by_slave[slave].push_back(claim); });

        std::for_each(stop_interfaces.begin(), stop_interfaces.end(), [&](auto &stop_interface)
                      {
                auto [slave, claim] = command_interface_map[stop_interface];
                stop_claims_by_slave[slave].push_back(claim); });

        std::vector<std::pair<std::string, hardware_interface::return_type>> slave_mode_switch_response;
        std::transform(slaves.begin(), slaves.end(), std::inserter(slave_mode_switch_response, slave_mode_switch_response.end()),
                       [&](auto &slave) -> std::pair<std::string, hardware_interface::return_type>
                       {
                           auto [name, instance] = slave;
                           return {name, instance->prepare_command_mode_switch(start_claims_by_slave[name], stop_claims_by_slave[name])};
                       });

        // TODO(bitmeal): remove debugging output
        std::for_each(slave_mode_switch_response.begin(), slave_mode_switch_response.end(),
                      [&](auto &slave_mode_switch_response)
                      {
                          RCLCPP_INFO(rclcpp::get_logger(__logger_name), "prepare command mode switch for %s -> %s",
                                      slave_mode_switch_response.first.c_str(),
                                      slave_mode_switch_response.second == hardware_interface::return_type::OK ? "OK" : "ERROR");
                      });

        if (std::find_if(slave_mode_switch_response.begin(), slave_mode_switch_response.end(),
                         [](auto &slave_mode_switch_response)
                         {
                             return slave_mode_switch_response.second != hardware_interface::return_type::OK;
                         }) == slave_mode_switch_response.end())
        {
            return hardware_interface::return_type::OK;
        }
        else
        {
            return hardware_interface::return_type::ERROR;
        }
    };

    hardware_interface::return_type ECClaimsResolver::perform_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces)
    {
        std::unordered_map<std::string, std::vector<std::string>> start_claims_by_slave;
        std::unordered_map<std::string, std::vector<std::string>> stop_claims_by_slave;

        std::for_each(start_interfaces.begin(), start_interfaces.end(), [&](auto &start_interface)
                      {
                auto [slave, claim] = command_interface_map[start_interface];
                start_claims_by_slave[slave].push_back(claim); });

        std::for_each(stop_interfaces.begin(), stop_interfaces.end(), [&](auto &stop_interface)
                      {
                auto [slave, claim] = command_interface_map[stop_interface];
                stop_claims_by_slave[slave].push_back(claim); });

        std::vector<std::pair<std::string, hardware_interface::return_type>> slave_mode_switch_response;
        std::transform(slaves.begin(), slaves.end(), std::inserter(slave_mode_switch_response, slave_mode_switch_response.end()),
                       [&](auto &slave) -> std::pair<std::string, hardware_interface::return_type>
                       {
                           auto [name, instance] = slave;
                           return {name, instance->perform_command_mode_switch(start_claims_by_slave[name], stop_claims_by_slave[name])};
                       });

        // TODO(bitmeal): remove debugging output
        std::for_each(slave_mode_switch_response.begin(), slave_mode_switch_response.end(),
                      [&](auto &slave_mode_switch_response)
                      {
                          RCLCPP_INFO(rclcpp::get_logger(__logger_name), "perform command mode switch for %s -> %s",
                                      slave_mode_switch_response.first.c_str(),
                                      slave_mode_switch_response.second == hardware_interface::return_type::OK ? "OK" : "ERROR");
                      });

        if (std::find_if(slave_mode_switch_response.begin(), slave_mode_switch_response.end(),
                         [](auto &slave_mode_switch_response)
                         {
                             return slave_mode_switch_response.second != hardware_interface::return_type::OK;
                         }) == slave_mode_switch_response.end())
        {
            return hardware_interface::return_type::OK;
        }
        else
        {
            return hardware_interface::return_type::ERROR;
        }
    };

} // namespace soem_driver
