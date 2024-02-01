#include <cmath>

#include "soem_driver_slave_interface/soem_driver_slave.hpp"
#include "soem_modules_trinamic/common.hpp"
#include "soem_modules_trinamic/tmcl.hpp"

#include "yaml-cpp/yaml.h"

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

        //// MODULE IMPLEMENTATION
        const std::string __logger_name = "soem_slave_modules/trinamic_tmcm1610";

        constexpr static uint64_t vendor_id = 0x0286;
        constexpr static uint64_t product_code = 0x0070;

        bool has_gripper = false;
        double gear_ratio = 0.0;
        double torque_constant = 0.0;

        std::vector<double> state_interfaces;
        std::vector<double> command_interfaces;

        bool init(std::unordered_map<std::string, std::string> /* parameters */)
        {
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 instantiated");

            return true;
        }

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces()
        {
            state_interfaces.resize(4, NAN);
            return {
                {"joint", "position", &state_interfaces[0]},
                {"joint", "velocity", &state_interfaces[1]},
                {"joint", "effort", &state_interfaces[2]},

                {"gripper", "position", &state_interfaces[3]}};
        };
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces()
        {
            command_interfaces.resize(4, NAN);
            return soem_driver::list_initialize_non_copyable_interface<hardware_interface::CommandInterface>(
                {{"joint", "position", &command_interfaces[0]},
                 {"joint", "velocity", &command_interfaces[1]},
                 {"joint", "effort", &command_interfaces[2]},

                 {"gripper", "position", &command_interfaces[3]}});
        };

        virtual bool configure(
            uint64_t vendor_id,
            uint64_t product_code,
            uint64_t /* revision_number */,
            std::unordered_map<std::string, std::string> parameters)
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

            if (parameters.find("gear_ratio") == parameters.end())
            {
                RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "no <gear_ratio> parameter found!");
                return false;
            }

            if (parameters.find("torque_constant") == parameters.end())
            {
                RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "no <torque_constant> parameter found!");
                return false;
            }

            torque_constant = std::stod(parameters["torque_constant"]);
            gear_ratio = std::stod(parameters["gear_ratio"]);

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Configuration OK! Vendor ID(0x%016lx), Product Code(0x%016lx)", this->vendor_id, this->product_code);
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Torque constant: %f [Nm/A]", torque_constant);
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Gear ratio: %f/1", gear_ratio);
            if (has_gripper)
            {
                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Module configured for gripper");
            }

            return true;
        };

        virtual void setup_SDO_hook(soem_driver::SDOwrite_t /* SDOwrite */)
        {
            // no SDO setup hook from PO --> SO necessary
            // RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 - call to: %s", __FUNCTION__);
        };

        // make data from TxPDO available in state interface
        virtual hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &)
        {
            // RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 - call to: %s", __FUNCTION__);
            
            // TODO(bitmeal): state machine to read config and home axes
            // GAP 250: Encoder steps per rotation
            // GAP 251: Encoder direction (invert movement direction of axis)
            
            
            TxPDO_t TxPDO_map;
            std::span<std::byte> TxPDO_map_buffer {reinterpret_cast<std::byte*>(&TxPDO_map), sizeof(TxPDO_map)};
            std::copy(TxPDO.begin(), TxPDO.end(), TxPDO_map_buffer.begin());

            // TODO(bitmeal): convert to actual required units
            state_interfaces[0] = TxPDO_map.position_ticks;
            state_interfaces[1] = TxPDO_map.vel_rpm;
            state_interfaces[2] = TxPDO_map.current_ma;

            // TODO(bitmeal): read gripper position if has gripper

            return hardware_interface::return_type::OK;
        };

        // write data from command interface to RxPDO
        virtual hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &)
        {
            // RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 - call to: %s", __FUNCTION__);
            return hardware_interface::return_type::OK;
        };
    };
} // namespace soem_slave_modules

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    soem_slave_modules::TrinamicTMCM1610, soem_driver_slave_interface::SOEMDriverSlave)
