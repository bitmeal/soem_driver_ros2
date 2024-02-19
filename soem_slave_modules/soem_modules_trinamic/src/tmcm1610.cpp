#include <cmath>

#include "soem_driver_slave_interface/soem_driver_slave.hpp"
#include "soem_modules_trinamic/common.hpp"
#include "soem_modules_trinamic/tmcl.hpp"

#include "yaml-cpp/yaml.h"

#include "hfsm2/machine.hpp"

#include "rclcpp/rclcpp.hpp"

namespace soem_slave_modules
{
    using namespace trinamic;

    class TrinamicTMCM1610 : public soem_driver_slave_interface::SOEMDriverSlave
    {
        //// TYPES AND CONSTANTS
        constexpr static uint8_t MBX_GGP_UserVarBank = 2;

        enum class MBX_TMCLModuleAddress : uint8_t
        {
            DRIVE = 0,
            GRIPPER = 1
        };

        // read with GAP 208 - TMC260 driver error flags
        enum class MBX_GripperDriverErrorFlags : uint8_t
        {
            STALL_GUARD_STATUS = 0x1,
            GRIPPER_OVER_TEMPERATURE = 0x2,
            PRE_WARNING_OVER_TEMPERATURE = 0x4,
            SHORT_TO_GROUND_A = 0x8,
            SHORT_TO_GROUND_B = 0x10,
            OPEN_LOAD_A = 0x20,
            OPEN_LOAD_B = 0x40,
            STAND_STILL = 0x80
        };

        /* // old module revision !?
        enum class StatusFlags : uint32_t
            OVER_CURRENT = 1,
            UNDER_VOLTAGE = 2,
            OVER_VOLTAGE = 4,
            OVER_TEMPERATURE = 8,
            HALTED = 16,
            HALL_SENSOR = 32,
            ENCODER = 64,
            MOTOR_WINDING = 128,
            CYCLE_TIME_VIOLATION = 256,
            INIT_SIN_COMM = 512,
        };
        */

        enum class EC_StatusFlags : uint32_t
        {
            OVER_CURRENT = 0x1,
            UNDER_VOLTAGE = 0x2,
            OVER_VOLTAGE = 0x4,
            OVER_TEMPERATURE = 0x8,
            MOTOR_HALTED = 0x10,
            HALL_SENSOR_ERROR = 0x20,
            ENCODER_ERROR = 0x40,
            //    INITIALIZATION_ERROR = 0x80,
            PWM_MODE_ACTIVE = 0x100,
            VELOCITY_MODE = 0x200,
            POSITION_MODE = 0x400,
            TORQUE_MODE = 0x800,
            //    EMERGENCY_STOP = 0x1000,
            //    FREERUNNING = 0x2000,
            POSITION_REACHED = 0x4000,
            INITIALIZED = 0x8000,
            ETHERCAT_TIMEOUT = 0x10000,
            I2T_EXCEEDED = 0x20000
        };

        struct RxPDO_t
        {
            int32_t setpoint;
            uint8_t mode;

            // padding
        private:
            int8_t _padding0[1];
            int16_t _padding1[1];
            int32_t _padding2[8];
        } __attribute__((__packed__));
        static_assert(sizeof(RxPDO_t) == 40);

        struct TxPDO_t
        {
            int32_t position_ticks; // encoder ticks
            int32_t current_ma;
            int32_t vel_rpm; // motor axis
            uint32_t status_flags;
            int32_t target_position_ticks; // encoder ticks
            int32_t target_current_ma;
            int32_t target_vel_rpm;
            int32_t ramp_gen_vel_rpm;

            // padding
        private:
            int32_t _padding0[2];
        } __attribute__((__packed__));
        static_assert(sizeof(TxPDO_t) == 40);

        struct MbxSend_t
        {
            uint8_t module_address; // drive / gripper
            uint8_t command;        // TMCL command number
            uint8_t parameter;      // type / paramter / address...
            uint8_t motor_bank;     // 0 for drive motor
            int32_t value;          // BIG ENDIAN!
        } __attribute__((__packed__));
        static_assert(sizeof(MbxSend_t) == 8);

        struct MbxReceive_t
        {
            uint8_t origin_address; // origin bus
            uint8_t module_address; // drive / gripper
            uint8_t status;         // TMCL status
            uint8_t command;        // response to what TMCL command number
            uint32_t value;         // BIG ENDIAN!
        } __attribute__((__packed__));
        static_assert(sizeof(MbxReceive_t) == 8);

        // state symbols
        enum COMMAND_MODE
        {
            NONE,
            EFFORT,
            VELOCITY,
            POSITION
        };

        enum DRIVER_STATE
        {
            STARTUP,
            WAITING_INITIALIZED_READ,
            UNINITIALIZED,
            WAITING_INITIALIZATION,
            INITIALIZED // RUNNING
        };

        //// MODULE IMPLEMENTATION
        const std::string __logger_name = "soem_slave_modules/trinamic_tmcm1610";

        COMMAND_MODE command_mode = NONE;
        DRIVER_STATE driver_state = STARTUP;

        // TODO(bitmeal): handle gripper
        std::string command_mode_to_claim(COMMAND_MODE command_mode) const
        {
            if (command_mode == POSITION)
                return "joint/position";
            if (command_mode == VELOCITY)
                return "joint/velocity";
            if (command_mode == EFFORT)
                return "joint/effort";

            return "<none>";
        };

        // TODO(bitmeal): handle gripper
        COMMAND_MODE claim_to_command_mode(const std::string &claim) const
        {
            if (claim == "joint/position")
                return POSITION;
            if (claim == "joint/velocity")
                return VELOCITY;
            if (claim == "joint/effort")
                return EFFORT;

            return NONE;
        };

        constexpr static uint64_t vendor_id = 0x0286;
        constexpr static uint64_t product_code = 0x0070;

        bool has_gripper = false;
        // double gear_ratio = 0.0;
        uint32_t encoder_ticks = 0;
        double torque_constant = .0;

        std::vector<double> state_interfaces;
        std::vector<double> command_interfaces;

        bool init(std::unordered_map<std::string, std::string> /* parameters */) override
        {
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 instantiated");

            // fetch_mailbox = true;

            return true;
        }

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override
        {
            state_interfaces.resize(4, NAN);
            return {
                {"joint", "position", &state_interfaces[0]},
                {"joint", "velocity", &state_interfaces[1]},
                {"joint", "effort", &state_interfaces[2]},

                {"gripper", "position", &state_interfaces[3]}};
        };
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
        {
            command_interfaces.resize(4, NAN);
            return soem_driver::list_initialize_non_copyable_interface<hardware_interface::CommandInterface>(
                {{"joint", "position", &command_interfaces[0]},
                 {"joint", "velocity", &command_interfaces[1]},
                 {"joint", "effort", &command_interfaces[2]},

                 {"gripper", "position", &command_interfaces[3]}});
        };

        bool configure(
            uint64_t vendor_id,
            uint64_t product_code,
            uint64_t revision_number,
            std::unordered_map<std::string, std::string> parameters) override
        {
            // TODO(bitmeal): error handling for type conversions
            // TODO(bitmeal): gear ratio configuration using transmissions

            if (!(vendor_id == this->vendor_id && product_code == this->product_code))
            {
                RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "Configuration failed! Expected: Vendor ID(0x%016lx), Product Code(0x%016lx); got: Vendor ID(0x%016lx), Product Code(0x%016lx)", this->vendor_id, this->product_code, vendor_id, product_code);
                return false;
            }

            if (parameters.find("gripper") != parameters.end())
            {
                // read truthy value using yaml library
                YAML::Node gripper_value = YAML::Load(parameters["gripper"]);
                if (gripper_value.IsScalar())
                {
                    has_gripper = gripper_value.as<bool>();
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger(__logger_name), "found <gripper> parameter but failed to read value as scalar!");
                }
            }

            // if (parameters.find("gear_ratio") == parameters.end())
            // {
            //     RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "no <gear_ratio> parameter found!");
            //     return false;
            // }

            if (parameters.find("torque_constant") == parameters.end())
            {
                RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "no <torque_constant> parameter found!");
                return false;
            }

            torque_constant = std::stod(parameters["torque_constant"]);
            // gear_ratio = std::stod(parameters["gear_ratio"]);

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Configuration OK! Vendor ID(0x%016lx), Product Code(0x%016lx), Product Revision(0x%016lx)", this->vendor_id, this->product_code, revision_number);
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Torque constant: %f [Nm/A]", torque_constant);
            // RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Gear ratio: %f/1", gear_ratio);

            if (has_gripper)
            {
                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Module configured for gripper");
            }

            return true;
        };

        void setup_SDO_hook(soem_driver::SDOwrite_t /* SDOwrite */) override{
            // no SDO setup hook from PO --> SO necessary
            // RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 - call to: %s", __FUNCTION__);
        };

        void log_command_mode_switch_interfaces_error(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces)
        {
            std::string start_interfaces_log;
            std::for_each(start_interfaces.begin(), start_interfaces.end(),
                          [&](auto &interface)
                          {
                              start_interfaces_log += interface;
                              if (interface != start_interfaces.back())
                              {
                                  start_interfaces_log += ", ";
                              }
                          });

            std::string stop_interfaces_log;
            std::for_each(stop_interfaces.begin(), stop_interfaces.end(),
                          [&](auto &interface)
                          {
                              stop_interfaces_log += interface;
                              if (interface != stop_interfaces.back())
                              {
                                  stop_interfaces_log += ", ";
                              }
                          });

            RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "cannot satisfy requested command mode switch! start claims: %s; stop claims: %s", start_interfaces_log.c_str(), stop_interfaces_log.c_str());
        }

        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override
        {
            // TODO(bitmeal): handle gripper

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 - call to: %s", __FUNCTION__);

            COMMAND_MODE next_command_mode = command_mode;

            // set command mode to none, if in stop interfaces/claims
            if (std::find(stop_interfaces.begin(), stop_interfaces.end(), command_mode_to_claim(next_command_mode)) != stop_interfaces.end())
            {
                next_command_mode = NONE;
            }

            // apply requested start interfaces/claims to next_command_mode
            for (auto &start_interface : start_interfaces)
            {
                auto start_mode = claim_to_command_mode(start_interface);
                if (
                    start_mode == next_command_mode ||
                    next_command_mode == NONE)
                {
                    next_command_mode = start_mode;
                }
                else
                {
                    log_command_mode_switch_interfaces_error(start_interfaces, stop_interfaces);
                    return hardware_interface::return_type::ERROR;
                }
            };

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "prepared command mode switch; next used claim: %s", command_mode_to_claim(next_command_mode).c_str());
            return hardware_interface::return_type::OK;
        };

        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override
        {
            // TODO(bitmeal): handle gripper

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 - call to: %s", __FUNCTION__);

            COMMAND_MODE next_command_mode = command_mode;

            // set command mode to none, if in stop interfaces/claims
            if (std::find(stop_interfaces.begin(), stop_interfaces.end(), command_mode_to_claim(next_command_mode)) != stop_interfaces.end())
            {
                next_command_mode = NONE;
            }

            // apply requested start interfaces/claims to next_command_mode
            for (auto &start_interface : start_interfaces)
            {
                auto start_mode = claim_to_command_mode(start_interface);
                if (
                    start_mode == next_command_mode ||
                    next_command_mode == NONE)
                {
                    next_command_mode = start_mode;
                }
                else
                {
                    log_command_mode_switch_interfaces_error(start_interfaces, stop_interfaces);
                    return hardware_interface::return_type::ERROR;
                }
            };

            // apply mode switch
            command_mode = next_command_mode;

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "switched command mode; used claim: %s", command_mode_to_claim(next_command_mode).c_str());
            return hardware_interface::return_type::OK;
        };

        diagnostic_msgs::msg::DiagnosticStatus build_diagnostics(const uint32_t pdo_status)
        {
            diagnostic_updater::DiagnosticStatusWrapper builder{};

            builder.set__name(__logger_name);

            // TODO(bitmeal): actual status
            builder.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");

            // TODO(bitmeal): state machine state

            builder.add("OVER_CURRENT", (bool)((uint32_t)EC_StatusFlags::OVER_CURRENT & pdo_status));
            builder.add("UNDER_VOLTAGE", (bool)((uint32_t)EC_StatusFlags::UNDER_VOLTAGE & pdo_status));
            builder.add("OVER_VOLTAGE", (bool)((uint32_t)EC_StatusFlags::OVER_VOLTAGE & pdo_status));
            builder.add("OVER_TEMPERATURE", (bool)((uint32_t)EC_StatusFlags::OVER_TEMPERATURE & pdo_status));
            builder.add("MOTOR_HALTED", (bool)((uint32_t)EC_StatusFlags::MOTOR_HALTED & pdo_status));
            builder.add("HALL_SENSOR_ERROR", (bool)((uint32_t)EC_StatusFlags::HALL_SENSOR_ERROR & pdo_status));
            builder.add("ENCODER_ERROR", (bool)((uint32_t)EC_StatusFlags::ENCODER_ERROR & pdo_status));
            // builder.add("INITIALIZATION_ERROR", (bool)((uint32_t)EC_StatusFlags::INITIALIZATION_ERROR & pdo_status));
            builder.add("PWM_MODE_ACTIVE", (bool)((uint32_t)EC_StatusFlags::PWM_MODE_ACTIVE & pdo_status));
            builder.add("VELOCITY_MODE", (bool)((uint32_t)EC_StatusFlags::VELOCITY_MODE & pdo_status));
            builder.add("POSITION_MODE", (bool)((uint32_t)EC_StatusFlags::POSITION_MODE & pdo_status));
            builder.add("TORQUE_MODE", (bool)((uint32_t)EC_StatusFlags::TORQUE_MODE & pdo_status));
            // builder.add("EMERGENCY_STOP", (bool)((uint32_t)EC_StatusFlags::EMERGENCY_STOP & pdo_status));
            // builder.add("FREERUNNING", (bool)((uint32_t)EC_StatusFlags::FREERUNNING & pdo_status));
            builder.add("POSITION_REACHED", (bool)((uint32_t)EC_StatusFlags::POSITION_REACHED & pdo_status));
            builder.add("INITIALIZED", (bool)((uint32_t)EC_StatusFlags::INITIALIZED & pdo_status));
            builder.add("ETHERCAT_TIMEOUT", (bool)((uint32_t)EC_StatusFlags::ETHERCAT_TIMEOUT & pdo_status));
            builder.add("I2T_EXCEEDED", (bool)((uint32_t)EC_StatusFlags::I2T_EXCEEDED & pdo_status));

            builder.add("encoder_ticks", encoder_ticks);

            return builder;
        };

        // make data from TxPDO available in state interface
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
        {
            mbx_consume_incoming([&](auto mbx_buffer)
                                 {
                // "deserialize" mailbox message
                MbxReceive_t mbx_msg;
                soem_driver::buffer mbx_msg_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&mbx_msg), sizeof(mbx_msg)};
                std::copy(mbx_buffer.begin(), mbx_buffer.end(), mbx_msg_buffer.begin());

                // act on message
                if(
                    mbx_msg.status == (uint8_t)TMCL::Status::SUCCESS &&
                    mbx_msg.module_address == (uint8_t)MBX_TMCLModuleAddress::DRIVE &&
                    mbx_msg.command == (uint8_t)TMCL::Command::GAP
                )
                {
                    encoder_ticks = from_big_endian(mbx_msg.value);
                    // fetch_mailbox = false;
                } });

            // RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 - call to: %s", __FUNCTION__);

            // TODO(bitmeal): state machine to read config and home axes
            // GAP 250: Encoder steps per rotation
            // GAP 251: Encoder direction (invert movement direction of axis)

            TxPDO_t TxPDO_map;
            soem_driver::buffer TxPDO_map_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&TxPDO_map), sizeof(TxPDO_map)};
            std::copy(TxPDO.begin(), TxPDO.end(), TxPDO_map_buffer.begin());

            // TODO(bitmeal): convert to actual required units
            state_interfaces[0] = TxPDO_map.position_ticks;
            state_interfaces[1] = TxPDO_map.vel_rpm;
            state_interfaces[2] = TxPDO_map.current_ma;

            // TODO(bitmeal): read gripper position if has gripper

            // make diagnostics info available
            diagnostics_status_queue.push(std::move(build_diagnostics(TxPDO_map.status_flags)));

            return hardware_interface::return_type::OK;
        };

        // write data from command interface to RxPDO
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
        {
            static bool req_encoder_ticks = true;
            if (req_encoder_ticks)
            {
                MbxSend_t mbx_get_encoder_ticks{
                    .module_address = (uint8_t)MBX_TMCLModuleAddress::DRIVE,
                    .command = (uint8_t)TMCL::Command::GAP,
                    .parameter = 250, // encoder ticks per revolution
                    .motor_bank = 0,
                    .value = 0 // ignored on GAP
                };
                soem_driver::buffer mbx_get_encoder_ticks_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&mbx_get_encoder_ticks), sizeof(mbx_get_encoder_ticks)};
                mbx_enqueue_send({mbx_get_encoder_ticks_buffer.begin(), mbx_get_encoder_ticks_buffer.end()}, true, true);
                req_encoder_ticks = false;
            }

            return hardware_interface::return_type::OK;
        };
    };
} // namespace soem_slave_modules

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    soem_slave_modules::TrinamicTMCM1610, soem_driver_slave_interface::SOEMDriverSlave)
