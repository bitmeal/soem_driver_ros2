
#ifndef SOEM_DRIVER__SOEM_DRIVER_HPP_
#define SOEM_DRIVER__SOEM_DRIVER_HPP_

#include <memory>
#include <vector>
#include <map>
#include <thread>
#include <chrono>
#include <atomic>

#include <tinyxml2.h>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "hardware_interface/system_interface.hpp"
#include "transmission_interface/transmission_loader.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "soem_driver_common/soem_driver_common.hpp"
#include "soem_driver_slave_interface/soem_driver_slave.hpp"
#include "soem_driver/soem_master.hpp"
#include "soem_driver/claims_resolver.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace soem_driver
{
    class SOEMDriver : public hardware_interface::SystemInterface
    {
    public:
        typedef struct EcSlavePluginInfo
        {
            std::string name;
            int alias;
            int position;
            std::string plugin_name;
            std::unordered_map<std::string, std::string> parameters;
            // std::vector<std::string> claims;
        } EcSlavePluginInfo;

        SOEMDriver();
        ~SOEMDriver();

        // CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        // methods ordered by expected order of execution during operation

        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;
        hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    private:
        std::string instance_name;
        std::string __logger_name;

        // member order for correct destruction, of all memers holding shared pointers to loaded libraries:
        //  |- slaves
        //  `-> initialized_slaves
        //    `-> claims_resolver
        pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave>
            slave_loader_{
                "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

        // map from: <slave name> --> <slave plugin instance>
        std::unordered_map<std::string, std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave>> slaves;
        std::vector<EcSlavePluginInfo> slave_infos;
        // map from: <joint> --> [<claim>]
        std::unordered_map<std::string, std::vector<std::string>> joint_claims;
        std::vector<std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave>> initialized_slaves;

        ECClaimsResolver claims_resolver;

        pluginlib::ClassLoader<transmission_interface::TransmissionLoader>
            transmission_loader_loader_{
                "transmission_interface", "transmission_interface::TransmissionLoader"};

        // map from: <slave name> --> {<state transmission instance>, <command transmission instance>}
        std::unordered_map<std::string, std::tuple<
                                            hardware_interface::TransmissionInfo,
                                            std::shared_ptr<transmission_interface::Transmission>,
                                            std::shared_ptr<transmission_interface::Transmission>>>
            transmissions;
        std::unordered_map<std::string, std::string> transmission_command_interface_map;
        std::vector<double> transmission_state_values;
        std::vector<double> transmission_command_values;

        // EtherCAT master configuration
        std::string ec_interface;
        std::chrono::microseconds ec_cycle_us;
        soem_master::SOEMMaster master;

        // diagnostics publisher
        class SOEMDiagnosticsPublisher : public rclcpp::Node
        {
        private:
            soem_master::SOEMMaster &master;
            const std::unordered_map<std::string, std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave>> &slaves;

            rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub;
            rclcpp::TimerBase::SharedPtr timer;

        public:
            SOEMDiagnosticsPublisher(
                const std::string name,
                const std::string interface,
                const rclcpp::NodeOptions &options,
                soem_master::SOEMMaster &soem_master,
                const std::unordered_map<std::string, std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave>> &soem_slaves,
                const std::chrono::milliseconds cycle_time_ms);

            ~SOEMDiagnosticsPublisher();

            // void task();
        };

        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> driver_ros_node_executor;
        std::shared_ptr<SOEMDiagnosticsPublisher> driver_ros_node;
        std::thread driver_ros_node_executor_thread;
        std::chrono::milliseconds diagnostics_cycle_ms;

        // void diagnostics_publisher_task(std::chrono::milliseconds cycle_time_ms);
        void run_diagnostics_publisher(std::chrono::milliseconds cycle_time_ms);
        void stop_diagnostics_publisher();

        // URDF hardware info parsing
        std::string _get_text_for_element(
            const tinyxml2::XMLElement *element_it, const std::string &tag_name);
        std::string _get_attribute_value(
            const tinyxml2::XMLElement *element_it, const char *attribute_name, std::string tag_name);
        std::string _get_attribute_value(
            const tinyxml2::XMLElement *element_it, const char *attribute_name, const char *tag_name);
        std::unordered_map<std::string, std::string> _parse_parameters_from_xml(
            const tinyxml2::XMLElement *params_it);
        const tinyxml2::XMLElement *_parse_hardware_from_doc(tinyxml2::XMLDocument &doc, const std::string name);
        std::vector<EcSlavePluginInfo> _parse_slaves_from_hardware(const tinyxml2::XMLElement *hardware);
        EcSlavePluginInfo _parse_slave_info(const tinyxml2::XMLElement *slave);
        std::unordered_map<std::string, std::vector<std::string>> _parse_joint_claims(const hardware_interface::HardwareInfo &info);
        void parse_hardware_info(const hardware_interface::HardwareInfo &info);
    };
} // namespace soem_driver

#endif // SOEM_DRIVER__SOEM_DRIVER_HPP_
