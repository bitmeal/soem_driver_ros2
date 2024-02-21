#include "soem_driver/soem_driver.hpp"
#include "soem_driver/alias_interface_factory.hpp"

#include "soem_driver/soem_master_helpers_ros.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

#include "yaml-cpp/yaml.h"

#include <numeric>

namespace soem_driver
{

    SOEMDriver::SOEMDriver() : // <guard linebreak>
                               hardware_interface::SystemInterface(),
                               instance_name({"generic"}),
                               driver_ros_node_executor_thread({}){};

    SOEMDriver::~SOEMDriver()
    {
        // cleanup if not traversing lifecycle correctly
        stop_diagnostics_publisher();

        // SOEM master will cleanup itself
    };

    // URDF parsing based on / taken from: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/component_parser.cpp

    /// Gets value of the text between tags.
    std::string SOEMDriver::_get_text_for_element(
        const tinyxml2::XMLElement *element_it, const std::string &tag_name)
    {
        const auto get_text_output = element_it->GetText();
        if (!get_text_output)
        {
            std::cerr << "text not specified in the " << tag_name << " tag" << std::endl;
            return "";
        }
        return get_text_output;
    }

    /// Gets value of the attribute on an XMLelement.
    std::string SOEMDriver::_get_attribute_value(
        const tinyxml2::XMLElement *element_it, const char *attribute_name, std::string tag_name)
    {
        const tinyxml2::XMLAttribute *attr;
        attr = element_it->FindAttribute(attribute_name);
        if (!attr)
        {
            throw std::runtime_error(
                "no attribute " + std::string(attribute_name) + " in " + tag_name + " tag");
        }
        return element_it->Attribute(attribute_name);
    }

    /// Gets value of the attribute on an XMLelement.
    std::string SOEMDriver::_get_attribute_value(
        const tinyxml2::XMLElement *element_it, const char *attribute_name, const char *tag_name)
    {
        return _get_attribute_value(element_it, attribute_name, std::string(tag_name));
    }

    /// Search XML snippet from URDF for parameters.
    std::unordered_map<std::string, std::string> SOEMDriver::_parse_parameters_from_xml(
        const tinyxml2::XMLElement *params_it)
    {
        std::unordered_map<std::string, std::string> parameters;
        const tinyxml2::XMLAttribute *attr;

        while (params_it)
        {
            // Fill the map with parameters
            attr = params_it->FindAttribute("name");
            if (!attr)
            {
                throw std::runtime_error("no parameter name attribute set in param tag");
            }
            const std::string parameter_name = params_it->Attribute("name");
            const std::string parameter_value = _get_text_for_element(params_it, parameter_name);
            parameters[parameter_name] = parameter_value;

            params_it = params_it->NextSiblingElement("param");
        }
        return parameters;
    }

    const tinyxml2::XMLElement *SOEMDriver::_parse_hardware_from_doc(tinyxml2::XMLDocument &doc, const std::string name)
    {
        tinyxml2::XMLElement *robot_it = doc.RootElement();
        if (std::string("robot").compare(robot_it->Name()))
        {
            throw std::runtime_error("the robot tag is not root element in URDF");
        }

        const tinyxml2::XMLElement *ros2_control_it = robot_it->FirstChildElement("ros2_control");
        if (!ros2_control_it)
        {
            throw std::runtime_error("no ros2_control tag");
        }

        // iterate over all ros2_control tags and find current hardware
        while (ros2_control_it)
        {
            if (!ros2_control_it->FindAttribute("name"))
            {
                throw std::runtime_error("no attribute name in ros2_control tag");
            }

            if (!name.compare(ros2_control_it->Attribute("name")))
            {
                break;
            }

            // not our hardware configuration; continue
            ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
        }

        // find hardware tag
        const auto *hardware_it = ros2_control_it->FirstChildElement("hardware");
        if (!hardware_it)
        {
            throw std::runtime_error("no hardware tag in ros2_control tag");
        }

        return hardware_it;
    }

    SOEMDriver::EcSlavePluginInfo SOEMDriver::_parse_slave_info(const tinyxml2::XMLElement *slave)
    {
        const auto alias_it = slave->FirstChildElement("alias");
        const auto position_it = slave->FirstChildElement("position");
        const auto plugin_it = slave->FirstChildElement("plugin");

        if (!(alias_it && position_it && plugin_it))
        {
            throw std::runtime_error("ec_slaves does not contain all mandatory tags alias, position and plugin");
        }

        const auto name = _get_attribute_value(slave, "name", "ec_slave");
        const auto alias = _get_text_for_element(alias_it, "alias");
        const auto position = _get_text_for_element(position_it, "position");
        const auto plugin_name = _get_text_for_element(plugin_it, "plugin");

        return {
            name,
            std::stoi(alias),
            std::stoi(position),
            plugin_name,
            _parse_parameters_from_xml(slave->FirstChildElement("param"))};
    };

    std::vector<SOEMDriver::EcSlavePluginInfo> SOEMDriver::_parse_slaves_from_hardware(const tinyxml2::XMLElement *hardware)
    {
        std::vector<EcSlavePluginInfo> slaves;

        const auto *ec_slave_it = hardware->FirstChildElement("ec_slave");
        while (ec_slave_it)
        {
            slaves.push_back(_parse_slave_info(ec_slave_it));

            ec_slave_it = ec_slave_it->NextSiblingElement("ec_slave");
        }

        return slaves;
    };

    std::unordered_map<std::string, std::vector<std::string>>
    SOEMDriver::_parse_joint_claims(const hardware_interface::HardwareInfo &info)
    {
        std::unordered_map<std::string, std::vector<std::string>> claims_map;

        for (auto &&joint : info.joints)
        {
            std::vector<std::string> claims_list;

            try
            {
                std::string claims_str = joint.parameters.at("ec_claims");
                RCLCPP_DEBUG(rclcpp::get_logger(__logger_name), "joint %s claims (raw): %s", joint.name.c_str(), claims_str.c_str());

                YAML::Node claims_yaml = YAML::Load(claims_str);

                auto record_claim = [&](const YAML::Node &claim)
                {
                    if (claim.IsScalar())
                    {
                        claims_list.push_back(claim.as<std::string>());
                    }
                    else
                    {
                        RCLCPP_WARN(rclcpp::get_logger(__logger_name),
                                    "failed to process a claim on joint %s; sequence of claims seems to include yaml structures other than scalars",
                                    joint.name.c_str());
                    }
                };

                if (claims_yaml.IsSequence())
                {
                    for (auto &&claim : claims_yaml)
                    {
                        record_claim(claim);
                    }
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger(__logger_name), "claims parameter for joint %s is no sequence", joint.name.c_str());
                    record_claim(claims_yaml);
                }
            }
            catch (const std::out_of_range &e)
            {
                RCLCPP_WARN(rclcpp::get_logger(__logger_name), "joint %s has no claims parameter", joint.name.c_str());
            }

            claims_map[joint.name] = claims_list;
        }

        return claims_map;
    };

    // based on: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/component_parser.cpp
    void SOEMDriver::parse_hardware_info(const hardware_interface::HardwareInfo &info)
    {
        const std::string &urdf = info.original_xml;

        // Check if everything OK with URDF string
        if (urdf.empty())
        {
            throw std::runtime_error("empty URDF passed to robot");
        }
        tinyxml2::XMLDocument doc;
        if (!doc.Parse(urdf.c_str()) && doc.Error())
        {
            throw std::runtime_error("invalid URDF passed in to robot parser");
        }
        if (doc.Error())
        {
            throw std::runtime_error("invalid URDF passed in to robot parser");
        }

        // get pointer to the matching hardware definition in URDF document
        const tinyxml2::XMLElement *hardware_it = _parse_hardware_from_doc(doc, info.name);

        // get diagnostics publisher loop cycle time configuration
        const auto *diagnostics_cycle_ms_it = hardware_it->FirstChildElement("diagnostics_cycle_ms");
        int ros_diagnostics_cycle_ms = 100;
        if (diagnostics_cycle_ms_it)
        {
            ros_diagnostics_cycle_ms = std::stoi(_get_text_for_element(diagnostics_cycle_ms_it, "diagnostics_cycle_ms"));
        }
        diagnostics_cycle_ms = std::chrono::milliseconds{ros_diagnostics_cycle_ms};
        RCLCPP_INFO(rclcpp::get_logger(__logger_name),
                    "configuring ROS diagnostics publisher cycle: %dms", ros_diagnostics_cycle_ms);

        // get interface configuration
        const auto *ec_interface_it = hardware_it->FirstChildElement("ec_interface");
        if (!ec_interface_it)
        {
            throw std::runtime_error("no ec_interface tag in hardware");
        }
        ec_interface = _get_text_for_element(ec_interface_it, "ec_interface");

        // get realtime loop cycle time configuration
        const auto *ec_cycle_us_it = hardware_it->FirstChildElement("ec_cycle_us");
        if (!ec_cycle_us_it)
        {
            throw std::runtime_error("no ec_cycle_us tag in hardware");
        }
        ec_cycle_us = std::chrono::microseconds{
            std::stoi(_get_text_for_element(ec_cycle_us_it, "ec_cycle_us"))};

        // get EtherCAT timeout configuration
        const auto *ec_timeout_pd_it = hardware_it->FirstChildElement("ec_timeout_pd_us");
        if (ec_timeout_pd_it)
        {
            int ec_timeout_pd = std::stoi(_get_text_for_element(ec_timeout_pd_it, "ec_timeout_pd_us"));
            RCLCPP_INFO(rclcpp::get_logger(__logger_name),
                        "configuring EtherCAT timeout for process data receive: %dµs", ec_timeout_pd);

            master.timeout_process_data = std::chrono::microseconds{ec_timeout_pd};
        }

        const auto *ec_timeout_mbx_recv_it = hardware_it->FirstChildElement("ec_timeout_mbxrx_us");
        if (ec_timeout_mbx_recv_it)
        {
            int ec_timeout_mbx_recv = std::stoi(_get_text_for_element(ec_timeout_mbx_recv_it, "ec_timeout_mbxrx_us"));
            RCLCPP_INFO(rclcpp::get_logger(__logger_name),
                        "configuring EtherCAT timeout for mailbox receive: %dµs", ec_timeout_mbx_recv);

            master.timeout_mbx_receive = std::chrono::microseconds{ec_timeout_mbx_recv};
        }

        const auto *ec_timeout_mbx_send_it = hardware_it->FirstChildElement("ec_timeout_mbxtx_us");
        if (ec_timeout_mbx_send_it)
        {
            int ec_timeout_mbx_send = std::stoi(_get_text_for_element(ec_timeout_mbx_send_it, "ec_timeout_mbxtx_us"));
            RCLCPP_INFO(rclcpp::get_logger(__logger_name),
                        "configuring EtherCAT timeout for mailbox send: %dµs", ec_timeout_mbx_send);

            master.timeout_mbx_send = std::chrono::microseconds{ec_timeout_mbx_send};
        }

        // get slave info/config
        slave_infos = _parse_slaves_from_hardware(hardware_it);

        // parse claims from joint configuration
        joint_claims = _parse_joint_claims(info);
    }

    // CallbackReturn SOEMDriver::on_error(const rclcpp_lifecycle::State &previous_state){};

    SOEMDriver::SOEMDiagnosticsPublisher::SOEMDiagnosticsPublisher(
        const std::string name,
        const std::string &interface,
        const rclcpp::NodeOptions &options,
        soem_master::SOEMMaster &master,
        const std::unordered_map<std::string, std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave>> &slaves,
        const std::chrono::milliseconds cycle_time_ms) : rclcpp::Node(name, options),
                                                         master(master),
                                                         slaves(slaves)
    {
        pub = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
        timer = create_wall_timer(cycle_time_ms, [&]
                                  {
            auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();

            // master diagnostics
            master.status_queue.consume_all([&](const auto master_status){
                auto master_diagnostics = build_master_status_diagnostics_ros(master_status);
                master_diagnostics.set__hardware_id(interface);
                msg->status.push_back(std::move(master_diagnostics));
            });

            // slave diagnostics
            std::for_each(slaves.begin(), slaves.end(), [&](auto& slave){
                auto& [_, slave_instance] = slave;
                slave_instance->diagnostics_status_queue.consume_all([&](auto slave_diag){
                    msg->status.push_back(std::move(slave_diag));
                });
            });

            msg->header.stamp = get_clock()->now();
            pub->publish(std::move(msg)); });
    };

    SOEMDriver::SOEMDiagnosticsPublisher::~SOEMDiagnosticsPublisher(){};

    void SOEMDriver::run_diagnostics_publisher(std::chrono::milliseconds cycle_time_ms)
    {
        // RCLCPP_DEBUG(rclcpp::get_logger(__logger_name), "Requested to run threaded diagnostics publisher Node; checking executor state");
        if (!driver_ros_node_executor || (driver_ros_node_executor && !driver_ros_node_executor->is_spinning()))
        {
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Starting diagnostics publisher thread");
            driver_ros_node_executor_thread = std::thread(
                [&]()
                {
                    // make executor
                    driver_ros_node_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

                    // diagnostics publisher
                    driver_ros_node = std::make_shared<SOEMDiagnosticsPublisher>(
                        "soem_driver_" + instance_name + "_diagnostics",
                        ec_interface,
                        rclcpp::NodeOptions(),
                        master,
                        slaves,
                        cycle_time_ms);

                    driver_ros_node_executor->add_node(driver_ros_node);
                    driver_ros_node_executor->spin();
                });

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Diagnostics publisher started");
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger(__logger_name), "Diagnostics publisher already running!");
        }
    };

    void SOEMDriver::stop_diagnostics_publisher()
    {
        if (driver_ros_node_executor_thread.joinable())
        {
            if (driver_ros_node_executor)
            {
                driver_ros_node_executor->cancel();
            }

            driver_ros_node_executor_thread.join();
        }

        // cleanup after canceling
        driver_ros_node_executor.reset();

        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Diagnostics publisher stopped");
    };

    // methods ordered by expected order of execution during operation

    CallbackReturn SOEMDriver::on_init(const hardware_interface::HardwareInfo &info)
    {
        // initialize driver
        hardware_interface::SystemInterface::on_init(info);

        instance_name = info.name;
        __logger_name = "SOEMDriver<" + instance_name + ">";
        RCLCPP_DEBUG(rclcpp::get_logger(__logger_name), "Initializing SOEMDriver");

        try
        {
            // non mandatory configuration parameters will be reported while parsing
            parse_hardware_info(info);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "failed to load with error: %s", e.what());
            return CallbackReturn::FAILURE;
        }

        // report ethercat interface
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "using interface %s", ec_interface.c_str());

        // report realtime loop cycle time
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "EtherCAT realtime loop will run at %ldµs cycle time", ec_cycle_us.count());

        // report slaves
        for (auto &&slave : slave_infos)
        {
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "slave %s<%s>@%i:%i", slave.name.c_str(), slave.plugin_name.c_str(), slave.alias, slave.position);
        }

        // report claims
        for (auto &&claims : joint_claims)
        {
            const std::string claims_str = std::accumulate(std::next(claims.second.begin()), claims.second.end(),
                                                           claims.second.front(),
                                                           [](std::string acc, std::string val)
                                                           { return acc + ", " + val; });

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "joint %s claims: [%s]", claims.first.c_str(), claims_str.c_str());
        }

        // load slave modules
        for (auto &&slave : slave_infos)
        {
            try
            {
                auto slave_plugin = slave_loader_.createSharedInstance(slave.plugin_name);
                // TODO(bitmeal): modify modules logger names
                // slave_plugin->__logger_name

                if (!slave_plugin->init(slave.parameters))
                {
                    RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "failed to load plugin %s for slave %s", slave.plugin_name.c_str(), slave.name.c_str());
                    return CallbackReturn::FAILURE;
                }

                slaves[slave.name] = slave_plugin;
                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "successfully loaded plugin %s for slave %s", slave.plugin_name.c_str(), slave.name.c_str());
            }
            catch (const std::exception &e)
            {
                // we don't throw, but load remaining plugins. slaves may have uninitialized plugins as result
                RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "failed to load plugin %s for slave %s; threw: %s", slave.plugin_name.c_str(), slave.name.c_str(), e.what());
            }
        }

        // TODO(bitmeal): allow generic transmissions in future
        // load transmissions
        // for now only:
        //  - SimpleTransmission (lazy specialization)
        //  - transmissions targeting one joint only in general
        for (auto &&transmission_info : info.transmissions)
        {
            if (
                transmission_info.type != "transmission_interface/SimpleTransmission" || transmission_info.joints.size() != 1)
            {

                RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "Ignoring unsupported transmission %s of type %s, targeting %ld joints!", transmission_info.name.c_str(), transmission_info.type.c_str(), transmission_info.joints.size());
                continue;
            }

            try
            {
                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "loading TransmissionLoader for transmission %s of type %s", transmission_info.name.c_str(), transmission_info.type.c_str());

                auto transmission_loader = transmission_loader_loader_.createUniqueInstance(transmission_info.type);

                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "loading Transmission for transmission %s of type %s", transmission_info.name.c_str(), transmission_info.type.c_str());
                auto transmission = std::make_pair(
                    transmission_loader->load(transmission_info),
                    transmission_loader->load(transmission_info));
                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "successfully loaded transmission %s of type %s for joint %s (loading twice for state/command)", transmission_info.name.c_str(), transmission_info.type.c_str(), transmission_info.joints[0].name.c_str());

                // add transmission to map from joint names to transmissions
                transmissions[transmission_info.joints[0].name] = transmission;
            }
            catch (const std::exception &e)
            {
                // we don't throw, but load remaining plugins. slaves may have uninitialized plugins as result
                RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "Failed to load Transmission or TransmissionLoader for transmission %s of type %s; threw: %s", transmission_info.name.c_str(), transmission_info.type.c_str(), e.what());
            }
        }

        // ECClaimsResolver claims_resolver{info, slaves, joint_claims};
        claims_resolver.__logger_name = __logger_name;
        claims_resolver.init(slaves, joint_claims);

        return CallbackReturn::SUCCESS;
    };

    CallbackReturn SOEMDriver::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
    {
        try
        {
            // initialize EtherCAT communications and discover slaves
            master.init(ec_interface);
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "master failed initialization: %s", e.what());

            return CallbackReturn::FAILURE;
        }

        // configure slaves with info from bus and attach SDO setup hooks
        for (auto &&slave_info : slave_infos)
        {
            auto slave_it = slaves.find(slave_info.name);
            if (slave_it == slaves.end())
            {
                RCLCPP_WARN(rclcpp::get_logger(__logger_name), "failed to lookup and configure plugin instance for slave %s", slave_info.name.c_str());
                continue;
            }

            auto &slave = *(slave_it->second);

            // find ethercat slaves from master by position or alias
            auto ec_slave_it = std::find_if(
                master.slaves.begin(), master.slaves.end(),
                [&](const soem_master::SOEMEcSlaveInfo &ec_slave)
                {
                    if (!slave_info.alias)
                    {
                        return ec_slave.position == slave_info.position;
                    }
                    else
                    {
                        return ec_slave.alias == slave_info.alias;
                    }
                });
            if (ec_slave_it == master.slaves.end())
            {
                RCLCPP_WARN(rclcpp::get_logger(__logger_name), "failed to find a slave on the bus for %s @%d:%d", slave_info.name.c_str(), slave_info.position, slave_info.alias);
                continue;
            }

            slave._position_ec_bus = ec_slave_it->position;
            // configure slave with position, alias, vendor id, product id, revision
            if (!slave.configure(
                    ec_slave_it->vendor_id,
                    ec_slave_it->product_code,
                    ec_slave_it->revision_number,
                    slave_info.parameters))
            {
                RCLCPP_WARN(rclcpp::get_logger(__logger_name), "slave %s reported failure when attempting configuration", slave_info.name.c_str());
                continue;
            }

            // attach SDO setup hook
            master.slave_attach_SDO_setup_hook(
                *ec_slave_it,
                [&](auto... Args)
                {
                    slave.setup_SDO_hook(Args...);
                });

            initialized_slaves.push_back(slave_it->second);

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "successfully configured slave %s", slave_info.name.c_str());
        }

        // bring up diagnostics publisher before bus start, to caputure early state
        run_diagnostics_publisher(diagnostics_cycle_ms);
        
        // start EtherCAT bus and setup data access structures
        master.start_bus();

        // assign data access structures to slaves
        for (auto slave_ptr : initialized_slaves)
        {
            auto ec_slave_da_it = std::find_if(
                master.slaves_data_access.begin(), master.slaves_data_access.end(),
                [&](const soem_master::SOEMEcSlaveDataAccess &ec_slave_da)
                {
                    return ec_slave_da.slave_info.position == slave_ptr->_position_ec_bus;
                });
            if (ec_slave_da_it == master.slaves_data_access.end())
            {
                // this should never happen... and check may be removed
                // RCLCPP_WARN(rclcpp::get_logger(__logger_name), "failed to find a slave on the bus for %s @%d:%d", slave_info.name.c_str(), slave_info.position, slave_info.alias);
                continue;
            }

            // setup data access
            slave_ptr->_RxPDO = ec_slave_da_it->RxPDO;
            slave_ptr->_TxPDO = ec_slave_da_it->TxPDO;

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "successfully configured process data access for slave in position %d", slave_ptr->_position_ec_bus);
        }

        return CallbackReturn::SUCCESS;
    };

    std::vector<hardware_interface::StateInterface> SOEMDriver::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        auto state_interfaces_by_joint = claims_resolver.export_state_interfaces_by_joint();

        size_t transmission_values_count = std::accumulate(state_interfaces_by_joint.begin(), state_interfaces_by_joint.end(), 0,
                                                           [&](size_t acc, auto &&single_joints_state_interfaces)
                                                           {
                                                               auto &[name, interfaces] = single_joints_state_interfaces;
                                                               return acc + (transmissions.find(name) != transmissions.end() ? interfaces.size() : 0);
                                                           });
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "reserving %zu state interface transmission variables", transmission_values_count);
        transmission_state_values.reserve(transmission_values_count);

        std::for_each(state_interfaces_by_joint.begin(), state_interfaces_by_joint.end(), [&](auto &&single_joints_state_interfaces)
                      {
                            auto& [name, interfaces] = single_joints_state_interfaces;
                            state_interfaces.reserve(state_interfaces.size() + interfaces.size());

                            // inject state transmissions
                            if(transmissions.find(name) != transmissions.end())
                            {
                                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "configuring state interface transmission for joint %s", name.c_str());

                                auto& state_transmission = transmissions[name].first;

                                // build new joint interfaces
                                // "proxy"-joint interfaces to the slaves interfaces are actuator interfaces
                                std::vector<hardware_interface::StateInterface> transmission_state_interfaces{};

                                std::transform(interfaces.begin(), interfaces.end(),
                                    std::back_inserter(transmission_state_interfaces),
                                    [&](auto& interface)
                                    {                                
                                        transmission_state_values.push_back({});
                                     
                                        return std::move(hardware_interface::StateInterface(
                                            interface.get_prefix_name(),
                                            interface.get_interface_name(),
                                            &transmission_state_values.back()
                                        ));
                                    }
                                    );

                                {
                                    std::vector<transmission_interface::JointHandle> state_transmission_joint_handles;
                                    std::transform(transmission_state_interfaces.begin(), transmission_state_interfaces.end(),
                                        std::back_inserter(state_transmission_joint_handles),
                                        [&](auto&& interface)
                                        {
                                            auto alias_factory = AliasInterfaceFactory{interface};
                                            return std::move(alias_factory.makeInterface<transmission_interface::JointHandle>(interface.get_prefix_name(), interface.get_interface_name()));
                                        });

                                    std::vector<transmission_interface::ActuatorHandle> state_transmission_actuator_handles;
                                    std::transform(interfaces.begin(), interfaces.end(),
                                        std::back_inserter(state_transmission_actuator_handles),
                                        [&](auto&& interface)
                                        {
                                            auto alias_factory = AliasInterfaceFactory{interface};
                                            return std::move(alias_factory.makeInterface<transmission_interface::ActuatorHandle>(interface.get_prefix_name(), interface.get_interface_name()));
                                        });

                                    state_transmission->configure(
                                        state_transmission_joint_handles,
                                        state_transmission_actuator_handles
                                    );
                                }

                                // add newly built interfaces to exports
                                std::move(std::begin(transmission_state_interfaces), std::end(transmission_state_interfaces), std::back_inserter(state_interfaces));
                            }
                            else
                            {
                                // add interfaces to exports
                                std::move(std::begin(interfaces), std::end(interfaces), std::back_inserter(state_interfaces));
                            } });

        return state_interfaces;
    };

    std::vector<hardware_interface::CommandInterface> SOEMDriver::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        auto command_interfaces_by_joint = claims_resolver.export_command_interfaces_by_joint();

        size_t transmission_values_count = std::accumulate(command_interfaces_by_joint.begin(), command_interfaces_by_joint.end(), 0,
                                                           [&](size_t acc, auto &&single_joints_command_interfaces)
                                                           {
                                                               auto &[name, interfaces] = single_joints_command_interfaces;
                                                               return acc + (transmissions.find(name) != transmissions.end() ? interfaces.size() : 0);
                                                           });
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "reserving %zu command interface transmission variables", transmission_values_count);
        transmission_command_values.reserve(transmission_values_count);

        std::for_each(command_interfaces_by_joint.begin(), command_interfaces_by_joint.end(), [&](auto &&single_joints_command_interfaces)
                      {
                            auto& [name, interfaces] = single_joints_command_interfaces;
                            command_interfaces.reserve(command_interfaces.size() + interfaces.size());

                            // inject state transmissions
                            if(transmissions.find(name) != transmissions.end())
                            {
                                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "configuring command interface transmission for joint %s", name.c_str());

                                auto& command_transmission = transmissions[name].second;

                                // build new joint interfaces
                                // "proxy"-joint interfaces to the slaves interfaces are actuator interfaces
                                std::vector<hardware_interface::CommandInterface> transmission_command_interfaces{};

                                std::transform(interfaces.begin(), interfaces.end(),
                                    std::back_inserter(transmission_command_interfaces),
                                    [&](auto& interface)
                                    {                                
                                        transmission_command_values.push_back({});
                                     
                                        return std::move(hardware_interface::CommandInterface(
                                            interface.get_prefix_name(),
                                            interface.get_interface_name(),
                                            &transmission_command_values.back()
                                        ));
                                    }
                                    );

                                {
                                    std::vector<transmission_interface::JointHandle> command_transmission_joint_handles;
                                    std::transform(transmission_command_interfaces.begin(), transmission_command_interfaces.end(),
                                        std::back_inserter(command_transmission_joint_handles),
                                        [&](auto&& interface)
                                        {
                                            auto alias_factory = AliasInterfaceFactory{interface};
                                            return std::move(alias_factory.makeInterface<transmission_interface::JointHandle>(interface.get_prefix_name(), interface.get_interface_name()));
                                        });

                                    std::vector<transmission_interface::ActuatorHandle> command_transmission_actuator_handles;
                                    std::transform(interfaces.begin(), interfaces.end(),
                                        std::back_inserter(command_transmission_actuator_handles),
                                        [&](auto&& interface)
                                        {
                                            auto alias_factory = AliasInterfaceFactory{interface};
                                            return std::move(alias_factory.makeInterface<transmission_interface::ActuatorHandle>(interface.get_prefix_name(), interface.get_interface_name()));
                                        });

                                    command_transmission->configure(
                                        command_transmission_joint_handles,
                                        command_transmission_actuator_handles
                                    );
                                }

                                // add newly built interfaces to exports
                                std::move(std::begin(transmission_command_interfaces), std::end(transmission_command_interfaces), std::back_inserter(command_interfaces));
                            }
                            else
                            {
                                // add interfaces to exports
                                std::move(std::begin(interfaces), std::end(interfaces), std::back_inserter(command_interfaces));
                            } });

        return command_interfaces;
    };

    CallbackReturn SOEMDriver::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
    {
        master.run(ec_cycle_us);

        return CallbackReturn::SUCCESS;
    };

    hardware_interface::return_type SOEMDriver::read(const rclcpp::Time &time, const rclcpp::Duration &duration)
    {
        // transfer process data to slaves buffers
        master.transfer_TxPDO();

        // make slaves read process data
        for (auto slave_ptr : initialized_slaves)
        {
            auto &slave_da = master.slaves_data_access[slave_ptr->_position_ec_bus - 1];
            //// fetch read mailbox
            // fetch mailbox content as long as master has pending messages
            if (
                // check pending
                slave_da.receive_mbx_has_unread.load()
                &&
                // transfer
                slave_ptr->mbx_receive_queue.push({slave_da.RMbx.begin(),
                                                   slave_da.RMbx.end()})
            )
            {
                // clear pending flag if transfer successful
                slave_da.receive_mbx_has_unread.store(false);
            }

            slave_ptr->read(time, duration);
        }

        std::for_each(transmissions.begin(), transmissions.end(), [&](auto &&joints_transmissions)
                      { joints_transmissions.second.first->actuator_to_joint(); });

        return hardware_interface::return_type::OK;
    };

    hardware_interface::return_type SOEMDriver::write(const rclcpp::Time &time, const rclcpp::Duration &duration)
    {
        std::for_each(transmissions.begin(), transmissions.end(), [&](auto &&joints_transmissions)
                      { joints_transmissions.second.second->joint_to_actuator(); });

        // make slaves write new process data and requeset mailbox sends
        for (auto slave_ptr : initialized_slaves)
        {
            auto &slave_da = master.slaves_data_access[slave_ptr->_position_ec_bus - 1];

            slave_ptr->write(time, duration);

            //// send mailbox
            if (!slave_da.send_mbx_request.load() && !slave_da.receive_mbx.load())
            { // no ongoing send operation
                slave_ptr->mbx_send_queue.consume_one([&](auto mbx_send_data)
                                                      {
                                                            auto [msg, read_response, read_mbx_raw] = mbx_send_data;
                                                            std::copy_n(msg.begin(), std::min(msg.size(), slave_da.SMbx.size()), slave_da.SMbx.begin()); 

                                                            slave_da.receive_mbx_raw.store(read_mbx_raw);
                                                            slave_da.receive_mbx.store(read_response);
                                                            slave_da.send_mbx_request.store(true); });
            }
        }

        // transfer new process data to bus from slave module working buffers
        master.transfer_RxPDO();

        return hardware_interface::return_type::OK;
    };

    hardware_interface::return_type SOEMDriver::prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces)
    {
        return claims_resolver.prepare_command_mode_switch(start_interfaces, stop_interfaces);
    };

    hardware_interface::return_type SOEMDriver::perform_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces)
    {
        return claims_resolver.perform_command_mode_switch(start_interfaces, stop_interfaces);
    };

    CallbackReturn SOEMDriver::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "deactivating driver");
        master.stop();
        return CallbackReturn::SUCCESS;
    };

    CallbackReturn SOEMDriver::on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "cleaning up");
        stop_diagnostics_publisher();
        return CallbackReturn::SUCCESS;
    };

    CallbackReturn SOEMDriver::on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "shutting down");
        master.stop();
        stop_diagnostics_publisher();
        return CallbackReturn::SUCCESS;
    };

} // namespace soem_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    soem_driver::SOEMDriver, hardware_interface::SystemInterface)
