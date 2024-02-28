#include <cmath>
#include <numbers>

#include "soem_driver_slave_interface/soem_driver_slave.hpp"
#include "soem_modules_trinamic/common.hpp"
#include "soem_modules_trinamic/tmcl.hpp"

#include "yaml-cpp/yaml.h"

// #define HFSM2_ENABLE_PLANS
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

        struct Version_t
        {
            int major;
            int minor;
        };

        // state symbols
        enum class COMMAND_MODE : uint8_t
        {
            STOP = 0,
            EFFORT = 6,
            VELOCITY = 2,
            POSITION = 1,
            INITIALIZE = 7,
            SET_REFERENCE = 4,
            NO_MORE_ACTION = 3
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
        std::string __logger_name = "soem_slave_modules/trinamic_tmcm1610";

        COMMAND_MODE command_mode = COMMAND_MODE::STOP;
        DRIVER_STATE driver_state = STARTUP;

        // TODO(bitmeal): handle gripper
        std::string command_mode_to_claim(COMMAND_MODE command_mode) const
        {
            if (command_mode == COMMAND_MODE::POSITION)
                return "drive/position";
            if (command_mode == COMMAND_MODE::VELOCITY)
                return "drive/velocity";
            if (command_mode == COMMAND_MODE::EFFORT)
                return "drive/effort";

            return "<none>";
        };

        // TODO(bitmeal): handle gripper
        COMMAND_MODE claim_to_command_mode(const std::string &claim) const
        {
            if (claim == "drive/position")
                return COMMAND_MODE::POSITION;
            if (claim == "drive/velocity")
                return COMMAND_MODE::VELOCITY;
            if (claim == "drive/effort")
                return COMMAND_MODE::EFFORT;

            return COMMAND_MODE::STOP;
        };

        constexpr static uint64_t vendor_id = 0x0286;
        constexpr static uint64_t product_code = 0x0070;

        bool has_gripper = false;

        Version_t firmware_version;
        uint16_t module_type;
        uint32_t encoder_ticks = 0;
        double torque_constant = .0;
        double homing_current = .0;

        std::vector<double> state_interfaces;
        std::vector<double> command_interfaces;

        // DRIVER STATE MACHINE (not module; only HW interaction handling)
#define hfsmS(s) struct s
#define hfsmS_t(s) struct s : FSM::State

        // states forward declacration
        hfsmS(OnStop);
        hfsmS(MotionC);
        hfsmS(Initialize);
        hfsmS(HomeAxis);
        hfsmS(ZeroAxisPosition);
        hfsmS(MotionRunC);
        hfsmS(Stop);
        hfsmS(Effort);
        hfsmS(Velocity);
        hfsmS(Position);
        hfsmS(ConfigureC);
        hfsmS(ReadFW);
        hfsmS(ReadEncoderTicks);
        hfsmS(ResetTimeout);
        hfsmS(OffStop);

        // state machine events
        struct fsmEvent_Read
        {
            TxPDO_t txpdo;
        };
        struct fsmEvent_Write
        {
        };
        struct fsmEvent_MBX_Msg
        {
            MbxReceive_t msg;
        };

        // machine context config
        using FSM_t = hfsm2::MachineT<hfsm2::Config::ContextT<TrinamicTMCM1610 &>>;

        // orthogonals: handling PDO write and MBX simultaneously
        using FSM_Ortho_Config =
            FSM_t::OrthogonalPeers<
                hfsmS(OnStop),
                FSM_t::Composite<hfsmS(ConfigureC),
                                 hfsmS(ReadFW),
                                 hfsmS(ReadEncoderTicks),
                                 hfsmS(ResetTimeout)>>;

        using FSM_Region_Motion_Run = FSM_t::Composite<hfsmS(MotionRunC),
                                                       hfsmS(Stop),
                                                       hfsmS(Effort),
                                                       hfsmS(Velocity),
                                                       hfsmS(Position)>;
        using FSM_Motion =
            FSM_t::Composite<hfsmS(MotionC),
                             hfsmS(Initialize),
                             hfsmS(HomeAxis),
                             hfsmS(ZeroAxisPosition),
                             FSM_Region_Motion_Run>;

        // using FSM_Ortho_Motion =
        //     FSM_t::OrthogonalPeers<
        //         FSM_t::Composite<hfsmS(MotionC),
        //                          hfsmS(Initialize),
        //                          FSM_Region_Motion_Run>,
        //         hfsmS(ResetTimeout)>;

        // machine
        using FSM = FSM_t::
            PeerRoot<
                FSM_Ortho_Config,
                FSM_Motion,
                hfsmS(OffStop)>;

        // states implementation
        hfsmS_t(Stop_t)
        {
            void react(const fsmEvent_Write &, EventControl &control)
            {
                RxPDO_t RxPDO_map;

                RxPDO_map.mode = (uint8_t)COMMAND_MODE::STOP;
                RxPDO_map.setpoint = 0;

                soem_driver::buffer RxPDO_map_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&RxPDO_map), sizeof(RxPDO_map)};
                std::copy(RxPDO_map_buffer.begin(), RxPDO_map_buffer.end(), control.context().RxPDO.begin());
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore other events
            using FSM::State::react;
        };

        struct OnStop : Stop_t
        {
            void enter(Control &control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM entering: %s", "OnStop");
            };
        };

        hfsmS_t(ConfigureC)
        {
            void enter(PlanControl & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Configuration] entering: %s", "Configuration Region");

                auto plan = control.plan();
                plan.change<ReadFW, ReadEncoderTicks>();
                plan.change<ReadEncoderTicks, ResetTimeout>();
            };

            void planSucceeded(FullControl & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Configuration] success");
                // control.changeTo<FSM_Ortho_Motion>();
                control.changeTo<MotionC>();
            };

            void planFailed(FullControl & control)
            {
                control.changeTo<OffStop>();
            };

            // // use empty update method
            // using FSM::State::update;
            // // ignore events
            // using FSM::State::react;
        };

        hfsmS_t(ReadFW)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Configuration] entering: %s", "Read Firmware Version");
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Requesting Firmware Version from Hardware; Sending MBX Request");

                MbxSend_t mbx_get_firmware_version{
                    .module_address = (uint8_t)MBX_TMCLModuleAddress::DRIVE,
                    .command = (uint8_t)TMCL::Command::FIRMWARE_VERSION,
                    .parameter = 1, // binary format
                    .motor_bank = 0,
                    .value = 0 // ignored on FIRMWARE_VERSION
                };
                soem_driver::buffer mbx_get_firmware_version_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&mbx_get_firmware_version), sizeof(mbx_get_firmware_version)};
                control.context().mbx_enqueue_send({mbx_get_firmware_version_buffer.begin(), mbx_get_firmware_version_buffer.end()}, true, true);
            };

            void react(const fsmEvent_MBX_Msg &msg_event, EventControl &control)
            {
                // act on message
                if (
                    msg_event.msg.status == (uint8_t)TMCL::Status::SUCCESS &&
                    msg_event.msg.module_address == (uint8_t)MBX_TMCLModuleAddress::DRIVE &&
                    msg_event.msg.command == (uint8_t)TMCL::Command::FIRMWARE_VERSION)
                {
                    // uint16_t module_type = from_big_endian(*reinterpret_cast<const uint16_t *>(&msg_event.msg.value));
                    uint16_t module_type = from_big_endian((uint16_t)msg_event.msg.value);
                    RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Got Module type from Hardware: %hu", module_type);

                    // uint16_t fw_version = from_big_endian(*reinterpret_cast<const uint16_t *>(&msg_event.msg.value));
                    uint16_t fw_version_major = (uint8_t)(msg_event.msg.value >> 16);
                    uint16_t fw_version_minor = (uint8_t)(msg_event.msg.value >> 24);
                    RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Got Firmware Version from Hardware: %u.%u", fw_version_major, fw_version_minor);

                    control.context().firmware_version = {fw_version_major, fw_version_minor};
                    control.context().module_type = module_type;

                    // next plan step
                    control.succeed();
                }
                else
                {
                    control.fail();
                }
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore other events
            using FSM::State::react;
        };

        hfsmS_t(ReadEncoderTicks)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Configuration] entering: %s", "Read Encoder Ticks");
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Requesting Encoder Ticks from Hardware; Sending MBX Request");

                MbxSend_t mbx_get_encoder_ticks{
                    .module_address = (uint8_t)MBX_TMCLModuleAddress::DRIVE,
                    .command = (uint8_t)TMCL::Command::GAP,
                    .parameter = 250, // encoder ticks per revolution
                    .motor_bank = 0,
                    .value = 0 // ignored on GAP
                };
                soem_driver::buffer mbx_get_encoder_ticks_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&mbx_get_encoder_ticks), sizeof(mbx_get_encoder_ticks)};
                control.context().mbx_enqueue_send({mbx_get_encoder_ticks_buffer.begin(), mbx_get_encoder_ticks_buffer.end()}, true, true);
            };

            void react(const fsmEvent_MBX_Msg &msg_event, EventControl &control)
            {
                // act on message
                if (
                    msg_event.msg.status == (uint8_t)TMCL::Status::SUCCESS &&
                    msg_event.msg.module_address == (uint8_t)MBX_TMCLModuleAddress::DRIVE &&
                    msg_event.msg.command == (uint8_t)TMCL::Command::GAP)
                {
                    uint32_t encoder_ticks = from_big_endian(msg_event.msg.value);
                    RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Got Encoder Ticks from Hardware: %u", encoder_ticks);
                    control.context().encoder_ticks = encoder_ticks;

                    // next plan step
                    control.succeed();
                }
                else
                {
                    control.fail();
                }
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore other events
            using FSM::State::react;
        };

        hfsmS_t(ResetTimeout)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Configuration] entering: %s", "Reset Timeout Watcher");
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Requesting EtherCAT timeout reset from Hardware; Sending MBX Request");

                MbxSend_t mbx_reset_timeout{
                    .module_address = (uint8_t)MBX_TMCLModuleAddress::DRIVE,
                    .command = (uint8_t)TMCL::Command::SAP,
                    .parameter = 158, // clear EtherCAT timeout flag
                    .motor_bank = 0,
                    .value = 0 // ignored
                };
                soem_driver::buffer mbx_reset_timeout_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&mbx_reset_timeout), sizeof(mbx_reset_timeout)};
                control.context().mbx_enqueue_send({mbx_reset_timeout_buffer.begin(), mbx_reset_timeout_buffer.end()}, false);
            };

            void react(const fsmEvent_Read &txpdo_read_event, EventControl &control)
            {
                if (!(bool)((uint32_t)EC_StatusFlags::ETHERCAT_TIMEOUT & txpdo_read_event.txpdo.status_flags))
                {
                    RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Reset EtherCAT timeout on Hardware");
                    control.succeed();
                }
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore other events
            using FSM::State::react;
        };

        // plans from initialization to cyclic motion execution region
        hfsmS_t(MotionC)
        {
            void enter(PlanControl & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "Motion Region");

                auto plan = control.plan();
                if (control.context().homing_current != .0)
                { // execute homing procedure
                    plan.change<Initialize, HomeAxis>();
                    plan.change<HomeAxis, ZeroAxisPosition>();
                    plan.change<ZeroAxisPosition, MotionRunC>();
                }
                else
                { // no homing
                    plan.change<Initialize, MotionRunC>();
                }
            };

            void planSucceeded(FullControl & control)
            {
                // should never reach
                control.changeTo<OffStop>();
            };

            void planFailed(FullControl & control)
            {
                control.changeTo<OffStop>();
            };

            // // use empty update method
            // using FSM::State::update;
            // // ignore events
            // using FSM::State::react;
        };

        hfsmS_t(Initialize)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "Initialize");
            };

            void react(const fsmEvent_Read &txpdo_read_event, EventControl &control)
            {
                if ((bool)((uint32_t)EC_StatusFlags::INITIALIZED & txpdo_read_event.txpdo.status_flags))
                {
                    RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Hardware initialized successfuly");
                    control.succeed();
                }
            };

            void react(const fsmEvent_Write &, EventControl &control)
            {
                RxPDO_t RxPDO_map;

                if (control.context().firmware_version.major >= 2)
                {
                    RxPDO_map.mode = (uint8_t)COMMAND_MODE::VELOCITY;
                    RxPDO_map.setpoint = 100;
                }
                else
                {
                    RxPDO_map.mode = (uint8_t)COMMAND_MODE::INITIALIZE;
                    RxPDO_map.setpoint = 0;
                }

                soem_driver::buffer RxPDO_map_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&RxPDO_map), sizeof(RxPDO_map)};
                std::copy(RxPDO_map_buffer.begin(), RxPDO_map_buffer.end(), control.context().RxPDO.begin());
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore events
            using FSM::State::react;
        };

        hfsmS_t(HomeAxis)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "Home Axis");
            };

            void react(const fsmEvent_Read &txpdo_read_event, EventControl &control)
            {
                if (std::fabs(txpdo_read_event.txpdo.current_ma) >= std::fabs(control.context().homing_current) * 1000.)
                {
                    RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Homed Axis successfuly");
                    control.succeed();
                }
            };

            void react(const fsmEvent_Write &, EventControl &control)
            {
                RxPDO_t RxPDO_map;

                RxPDO_map.mode = (uint8_t)COMMAND_MODE::VELOCITY;
                RxPDO_map.setpoint = (int32_t)std::copysign(100, control.context().homing_current);

                soem_driver::buffer RxPDO_map_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&RxPDO_map), sizeof(RxPDO_map)};
                std::copy(RxPDO_map_buffer.begin(), RxPDO_map_buffer.end(), control.context().RxPDO.begin());
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore events
            using FSM::State::react;
        };

        hfsmS_t(ZeroAxisPosition)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "Zero Axis Position");
            };

            void react(const fsmEvent_Read &txpdo_read_event, EventControl &control)
            {
                // account for small deviations from residual movements and don't require exact 0
                if (std::fabs(txpdo_read_event.txpdo.position_ticks) < 2)
                {
                    RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Set Axis Position = 0 successfuly");
                    control.succeed();
                }
            };

            void react(const fsmEvent_Write &, EventControl &control)
            {
                RxPDO_t RxPDO_map;

                RxPDO_map.mode = (uint8_t)COMMAND_MODE::SET_REFERENCE;
                RxPDO_map.setpoint = 0;

                soem_driver::buffer RxPDO_map_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&RxPDO_map), sizeof(RxPDO_map)};
                std::copy(RxPDO_map_buffer.begin(), RxPDO_map_buffer.end(), control.context().RxPDO.begin());
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore events
            using FSM::State::react;
        };

        // cyclic motion execution region; switches state to current command mode on update() calls
        hfsmS_t(MotionRunC)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "Motion Run Region");
            };

            void update(FullControl & control)
            {
                switch (control.context().command_mode)
                {
                case COMMAND_MODE::EFFORT:
                    control.changeTo<Effort>();
                    break;
                case COMMAND_MODE::VELOCITY:
                    control.changeTo<Velocity>();
                    break;
                case COMMAND_MODE::POSITION:
                    control.changeTo<Position>();
                    break;
                default:
                    control.changeTo<Stop>();
                }
            };

            // // ignore events
            // using FSM::State::react;
        };

        struct Stop : Stop_t
        {
            void enter(Control &control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "Stop");
            };
        };

        hfsmS_t(Effort)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "Effort");
            };

            void react(const fsmEvent_Write &, EventControl &control)
            {
                RxPDO_t RxPDO_map;

                RxPDO_map.mode = (uint8_t)COMMAND_MODE::EFFORT;
                // torque_constant [Nm/A]
                // setpoint [mA]
                // command_interface [Nm]
                // setpoint = command_interface / torque_constant * 1000
                RxPDO_map.setpoint = control.context().command_interfaces[2] / control.context().torque_constant * 1000;

                soem_driver::buffer RxPDO_map_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&RxPDO_map), sizeof(RxPDO_map)};
                std::copy(RxPDO_map_buffer.begin(), RxPDO_map_buffer.end(), control.context().RxPDO.begin());
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore other events
            using FSM::State::react;
        };

        hfsmS_t(Velocity)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "Velocity");
            };

            void react(const fsmEvent_Write &, EventControl &control)
            {
                RxPDO_t RxPDO_map;

                RxPDO_map.mode = (uint8_t)COMMAND_MODE::VELOCITY;
                // setpoint [rpm]|[1/min] (motor spindle)
                // command_interface [rad/s]
                // setpoint = command_interface * 60 / ( 2 * PI )
                RxPDO_map.setpoint = control.context().command_interfaces[1] * 60 / (2 * std::numbers::pi);

                soem_driver::buffer RxPDO_map_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&RxPDO_map), sizeof(RxPDO_map)};
                std::copy(RxPDO_map_buffer.begin(), RxPDO_map_buffer.end(), control.context().RxPDO.begin());
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore other events
            using FSM::State::react;
        };

        hfsmS_t(Position)
        {
            void enter(Control & control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "Position");
            };

            void react(const fsmEvent_Write &, EventControl &control)
            {
                RxPDO_t RxPDO_map;

                RxPDO_map.mode = (uint8_t)COMMAND_MODE::POSITION;
                // setpoint [ticks] (encoder ticks)
                // command_interface [rad]
                // setpoint = command_interface * encoder_ticks / ( 2 * PI )
                RxPDO_map.setpoint = control.context().command_interfaces[0] * control.context().encoder_ticks / (2 * std::numbers::pi);

                soem_driver::buffer RxPDO_map_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&RxPDO_map), sizeof(RxPDO_map)};
                std::copy(RxPDO_map_buffer.begin(), RxPDO_map_buffer.end(), control.context().RxPDO.begin());
            };

            // // use empty update method
            // using FSM::State::update;
            // ignore other events
            using FSM::State::react;
        };

        struct OffStop : Stop_t
        {
            void enter(Control &control)
            {
                RCLCPP_INFO(rclcpp::get_logger(control.context().__logger_name), "Driver FSM [Motion] entering: %s", "OffStop");
            };
        };

        // instance
        std::unique_ptr<FSM::Instance> fsm;

#undef hfsmS_t
#undef hfsmS

        // SOEM MODULE IMPLEMENTATION
        bool init(const std::string& name, std::unordered_map<std::string, std::string> /* parameters */) override
        {
            __logger_name += "<" + name + ">";
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "trinamic_tmcm1610 module driver instantiated");

            return true;
        }

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override
        {
            state_interfaces.resize(4, NAN);
            return {
                {"drive", "position", &state_interfaces[0]},
                {"drive", "velocity", &state_interfaces[1]},
                {"drive", "effort", &state_interfaces[2]},

                {"gripper", "position", &state_interfaces[3]}};
        };
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
        {
            command_interfaces.resize(4, NAN);
            return soem_driver::list_initialize_non_copyable_interface<hardware_interface::CommandInterface>(
                {{"drive", "position", &command_interfaces[0]},
                 {"drive", "velocity", &command_interfaces[1]},
                 {"drive", "effort", &command_interfaces[2]},

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

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Vendor ID(0x%016lx), Product Code(0x%016lx), Product Revision(0x%016lx)", this->vendor_id, this->product_code, revision_number);

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

            if (parameters.find("torque_constant") == parameters.end())
            {
                RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "no <torque_constant> parameter found!");
                return false;
            }
            torque_constant = std::stod(parameters["torque_constant"]);
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Torque constant: %f [Nm/A]", torque_constant);

            if (parameters.find("homing_current_limit") != parameters.end())
            {
                homing_current = std::stod(parameters["homing_current_limit"]);
                if (homing_current != .0)
                {
                    RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Axis Homing configured; Current Limit abs: %f [A]; Direction: %s", std::fabs(homing_current), homing_current > 0 ? "pos(+)" : "neg(-)");
                }
            }

            if (has_gripper)
            {
                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Module configured for gripper");
            }

            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Configuration OK!");
            fsm = std::make_unique<FSM::Instance>(*this);

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

            // set command mode to stop, if in stop interfaces/claims
            if (std::find(stop_interfaces.begin(), stop_interfaces.end(), command_mode_to_claim(next_command_mode)) != stop_interfaces.end())
            {
                next_command_mode = COMMAND_MODE::STOP;
            }

            // apply requested start interfaces/claims to next_command_mode
            for (auto &start_interface : start_interfaces)
            {
                auto start_mode = claim_to_command_mode(start_interface);
                if (
                    start_mode == next_command_mode ||
                    next_command_mode == COMMAND_MODE::STOP)
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

            // set command mode to stop, if in stop interfaces/claims
            if (std::find(stop_interfaces.begin(), stop_interfaces.end(), command_mode_to_claim(next_command_mode)) != stop_interfaces.end())
            {
                next_command_mode = COMMAND_MODE::STOP;
            }

            // apply requested start interfaces/claims to next_command_mode
            for (auto &start_interface : start_interfaces)
            {
                auto start_mode = claim_to_command_mode(start_interface);
                if (
                    start_mode == next_command_mode ||
                    next_command_mode == COMMAND_MODE::STOP)
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
            builder.add("torque_constant", torque_constant);
            builder.add("module_type", module_type);
            builder.addf("firmware_version", "%u.%u", firmware_version.major, firmware_version.minor);

            return builder;
        };

        // make data from TxPDO available in state interface
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
        {
            // read process data from slave
            TxPDO_t TxPDO_map;
            soem_driver::buffer TxPDO_map_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&TxPDO_map), sizeof(TxPDO_map)};
            std::copy(TxPDO.begin(), TxPDO.end(), TxPDO_map_buffer.begin());

            // position_ticks [ticks] (encoder ticks)
            // state_interface [rad]
            // state_interface = position_ticks * 2 * PI  / encoder_ticks
            state_interfaces[0] = TxPDO_map.position_ticks * 2 * std::numbers::pi / encoder_ticks;
            // vel_rpm [rpm]|[1/min] (motor spindle)
            // state_interface [rad/s]
            // state_interface = vel_rpm * 2 * PI / 60
            state_interfaces[1] = TxPDO_map.vel_rpm * 2 * std::numbers::pi / 60;
            // torque_constant [Nm/A]
            // current_ma [mA]
            // state_interface [Nm]
            // state_interface = current_ma * torque_constant / 1000
            state_interfaces[2] = TxPDO_map.current_ma * torque_constant / 1000;

            // TODO(bitmeal): read gripper position if has gripper

            // make state machine react to hardware state & update state machine
            fsm->react(fsmEvent_Read{TxPDO_map});

            mbx_consume_incoming([&](auto mbx_buffer)
                                 {
                                     // "deserialize" mailbox message
                                     MbxReceive_t mbx_msg;
                                     soem_driver::buffer mbx_msg_buffer{reinterpret_cast<soem_driver::buffer::pointer>(&mbx_msg), sizeof(mbx_msg)};
                                     std::copy(mbx_buffer.begin(), mbx_buffer.end(), mbx_msg_buffer.begin());
                                 
                                    fsm->react(fsmEvent_MBX_Msg{mbx_msg}); });

            fsm->update();

            // make diagnostics info available
            diagnostics_status_queue.push(std::move(build_diagnostics(TxPDO_map.status_flags)));

            return hardware_interface::return_type::OK;
        };

        // write data from command interface to RxPDO
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
        {
            // update state machine and react to write event
            fsm->update();
            fsm->react(fsmEvent_Write{});

            return hardware_interface::return_type::OK;
        };
    };
} // namespace soem_slave_modules

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    soem_slave_modules::TrinamicTMCM1610, soem_driver_slave_interface::SOEMDriverSlave)
