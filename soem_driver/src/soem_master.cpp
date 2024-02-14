#include "soem_driver/soem_master.hpp"

#include <exception>
#include <ranges>
#include <limits.h>
#include <algorithm>
#include <functional>

#include "rclcpp/rclcpp.hpp"

// mlockall
#include <sys/mman.h>
// sched_setscheduler
#include <sched.h>
// getpid
#include <unistd.h>
// setpriority
#include <sys/resource.h>

namespace soem_master
{
    const std::string ec_state_to_string(/* ec_state */ uint16_t state)
    {
        switch (state)
        {
        case EC_STATE_NONE:
            return "NONE";
        case EC_STATE_INIT:
            return "INIT";
        case EC_STATE_PRE_OP:
            return "PRE_OP";
        case EC_STATE_BOOT:
            return "BOOT";
        case EC_STATE_SAFE_OP:
            return "SAFE_OP";
        case EC_STATE_OPERATIONAL:
            return "OP";
        // case EC_STATE_ACK:
        //     return "ACK";
        case EC_STATE_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
        }
    };

    // store context to master mappings to resolve from c-style non member function pointers
    std::map<ecx_contextt *, std::reference_wrapper<SOEMMaster>> SOEMCtxMasterMapping;

    // resolve context
    template <auto Callee, typename... Args>
    auto fnptr_call_SOEMMaster_member(ecx_contextt *ecx_context, Args... args)
    {
        return std::bind(Callee, &(SOEMCtxMasterMapping.at(ecx_context).get()), args...)();
    };
    // template <auto Callee, typename... Args>
    // auto caller(Args &&...args)
    // {
    //     // return Callee(std::forward<Args>(args)...);
    //     return std::bind(Callee, args...)();
    // };

    SOEMMaster::SOEMMaster()
        : timeout_process_data(EC_TIMEOUTRET),
          timeout_mbx_send(EC_TIMEOUTTXM),
          timeout_mbx_receive(EC_TIMEOUTRXM),
          __logger_name("soem_master"),
          slaves(_slaves),
          slaves_data_access(_slaves_data_access),
          status({.error_count = 0,
                  .rx_error_count = 0,
                  .tx_error_count = 0,
                  .rx_error_count_consecutive = 0,
                  .tx_error_count_consecutive = 0,
                  .other_ec_error_count_consecutive = 0,
                  .missed_cycle_deadline_count = 0,
                  .state = SOEMMaster::SOEMMasterState::UNINITIALIZED}),
          IOmap({}),
          RxPDO_working({}),
          RxPDO_transfer(false),
          TxPDO_working({}),
          TxPDO_transfer(false),
          SMbx((std::byte *)EC_SMbx, sizeof(EC_SMbx)),
          SMbx_working({}),
          RMbx((std::byte *)EC_RMbx, sizeof(EC_RMbx)),
          RMbx_working({}),
          ecx_context({
              &ecx_port,         // .port          =
              &ec_slave[0],      // .slavelist     =
              &ec_slavecount,    // .slavecount    =
              EC_MAXSLAVE,       // .maxslave      =
              &ec_group[0],      // .grouplist     =
              EC_MAXGROUP,       // .maxgroup      =
              &ec_esibuf[0],     // .esibuf        =
              &ec_esimap[0],     // .esimap        =
              0,                 // .esislave      =
              &ec_elist,         // .elist         =
              &ec_idxstack,      // .idxstack      =
              &EcatError,        // .ecaterror     =
              0,                 // .DCtO          =
              0,                 // .DCl           =
              &ec_DCtime,        // .DCtime        =
              &ec_SMcommtype[0], // .SMcommtype    =
              &ec_PDOassign[0],  // .PDOassign     =
              &ec_PDOdesc[0],    // .PDOdesc       =
              &ec_SM,            // .eepSM         =
              &ec_FMMU,          // .eepFMMU       =
              NULL,              // .FOEhook()
              NULL,              // .EOEhook()
              0                  // .manualstatechange
          }),
          is_init(false),
          cycle_counter(0),
          cyclic_master({}),
          cyclic_master_terminate(false),
          cyclic_master_running(false)

    {
        // register to resolve instance from context pointer
        SOEMCtxMasterMapping.insert(std::pair<ecx_contextt *, std::reference_wrapper<SOEMMaster>>(&ecx_context, std::ref(*this)));
    };
    SOEMMaster::~SOEMMaster()
    {
        deinit();

        // unregister for resolution from context pointer
        SOEMCtxMasterMapping.erase(&ecx_context);
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "unregistered SOEM master instance: %ld left", SOEMCtxMasterMapping.size());
    };

    // initialize interface and scan bus for slaves
    void SOEMMaster::init(const std::string &interface)
    {
        // init and open interface
        if (ecx_init(&ecx_context, interface.c_str()))
        {
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "initialized EtherCAT communication on interface %s", interface.c_str());

            // find and auto_configure slaves to PRE_OP
            if (ecx_config_init(&ecx_context, FALSE) > 0)
            {
                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "found %d slaves", ec_slavecount);
                ecx_readstate(&ecx_context);
                ec_log_slaves();

                // indexing at 1 with bus addresses; slave[0] is whole group/bus
                for (auto slave_idx : std::ranges::views::iota(1, ec_slavecount + 1))
                {
                    _slaves.push_back({
                        .vendor_id = ec_slave[slave_idx].eep_man,
                        .product_code = ec_slave[slave_idx].eep_id,
                        .revision_number = ec_slave[slave_idx].eep_rev,

                        // .position = ec_slave[slave_idx].configadr,
                        .position = slave_idx,
                        .alias = ec_slave[slave_idx].aliasadr,

                        .name = ec_slave[slave_idx].name,

                        .RxPDO_size = (size_t)(ec_slave[slave_idx].Obits / CHAR_WIDTH) + ec_slave[slave_idx].Obits % CHAR_WIDTH,
                        .TxPDO_size = (size_t)(ec_slave[slave_idx].Ibits / CHAR_WIDTH) + ec_slave[slave_idx].Ibits % CHAR_WIDTH,
                    });
                }
            }
            else
            {
                throw std::runtime_error("could not discover EtherCAT bus on interface " + interface);
            }
        }
        else
        {
            throw std::runtime_error("could not initialize EtherCAT communication on interface " + interface);
        }

        if (!cyclic_master_running.load())
        {
            // guard against races; may lead to incorrectly reported state
            status.state = SOEMMasterState::INITIALIZED;
            status_queue.push(status);
        }

        is_init = true;
    };

    void SOEMMaster::deinit()
    {
        // stop and join thread
        stop();

        if (is_init)
        {
            bus_down_SAFE_OP();
            ecx_close(&ecx_context);
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "closed EtherCAT communication");
        }
        
        status.state = SOEMMasterState::UNINITIALIZED;
    };

    // get all devices to SAFE_OP state, calling attached SDO setup hooks on transition from PO to SO
    void SOEMMaster::start_bus(size_t IOmap_size)
    {

        if (!cyclic_master_running.load())
        {
            // guard against races; may lead to incorrectly reported state
            status.state = SOEMMasterState::STARTING;
            status_queue.push(status);
        }

        // setup IOmap
        // slave input and output sizes are only available after mapping and PO2SO hooks!
        // size_t IOmap_size = std::accumulate(_slaves.begin(), _slaves.end(), 0,
        //                                     [&](size_t acc, SOEMEcSlaveInfo slave)
        //                                     {
        //                                         return acc + slave.RxPDO_size + slave.TxPDO_size;
        //                                     });

        IOmap.resize(IOmap_size, std::byte(0x00));
        ecx_config_overlap_map_group(&ecx_context, IOmap.data(), 0);

        // setup RxPDO/TxPDO non-owning buffer types and working buffers, copy IOmap content to buffers
        RxPDO_IOmap = {(std::byte *)(void *)(ec_group[0].outputs), ec_group[0].Obytes};
        RxPDO_working.resize(RxPDO_IOmap.size());
        std::copy(RxPDO_IOmap.begin(), RxPDO_IOmap.end(), RxPDO_working.begin());

        TxPDO_IOmap = {(std::byte *)(void *)(ec_group[0].inputs), ec_group[0].Ibytes};
        TxPDO_working.resize(TxPDO_IOmap.size());
        std::copy(TxPDO_IOmap.begin(), TxPDO_IOmap.end(), TxPDO_working.begin());

        // setup mailbox buffers and slave data access structures
        SMbx_working.resize(slaves.size(), {});
        RMbx_working.resize(slaves.size(), {});
        // _slaves_data_access.reserve(slaves.size());
        for (auto &&slave : _slaves)
        {
            SMbx_working[slave.position - 1].resize(ec_slave[slave.position].mbx_l, std::byte(0x00));
            RMbx_working[slave.position - 1].resize(ec_slave[slave.position].mbx_rl, std::byte(0x00));
            _slaves_data_access.emplace_back(slave, getRxPDO(slave), getTxPDO(slave), getSMbx(slave), getRMbx(slave));
        }

        /* wait for all slaves to reach SAFE_OP state */
        ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        /* configure DC options for every DC capable slave found in the list */
        ecx_configdc(&ecx_context);

        // /* read individual slave state and store in ec_slave[] */
        ecx_readstate(&ecx_context);
        if (ec_slave[0].state != EC_STATE_SAFE_OP)
        {
            if (!cyclic_master_running.load())
            {
                // guard against races; may lead to incorrectly reported state
                status.state = SOEMMasterState::ERROR;
                status_queue.push(status);
            }
            RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "NOT all slaves reached SAFE OP");
            ec_log_slaves();
            throw std::runtime_error("NOT all slaves reached SAFE OP");
        }

        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "all slaves reached SAFE OP; DC configured if capable");
        ec_log_slaves();
    };

    void SOEMMaster::bus_up_OP()
    {
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "requesting OP state for all slaves");

        // request OP state and wait for slaves to reach
        ec_slave[0].state = EC_STATE_OPERATIONAL;

        ecx_send_overlap_processdata_group(&ecx_context, 0);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
        ecx_writestate(&ecx_context, 0);

        int max_OP_PD = 200;
        /* wait for all slaves to reach OP state */
        do
        {
            ecx_send_overlap_processdata_group(&ecx_context, 0);
            ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
            ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 50000);
            // ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);
        } while (max_OP_PD-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
        ecx_readstate(&ecx_context);

        if (ec_slave[0].state != EC_STATE_OPERATIONAL)
        {
            if (!cyclic_master_running.load())
            {
                // guard against races; may lead to incorrectly reported state
                status.state = SOEMMasterState::ERROR;
                status_queue.push(status);
            }
            RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "NOT all slaves reached OP");
            ec_log_slaves();
            throw std::runtime_error("NOT all slaves reached OP");
        }

        if (!cyclic_master_running.load())
        {
            // guard against races; may lead to incorrectly reported state
            status.state = SOEMMasterState::IDLE;
            status_queue.push(status);
        }
    };

    void SOEMMaster::bus_down_SAFE_OP()
    {

        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Requesting SAFE_OP for all slaves");
        ec_slave[0].state = EC_STATE_SAFE_OP;
        ecx_writestate(&ecx_context, 0);

        if (!cyclic_master_running.load())
        {
            // guard against races; may lead to incorrectly reported state
            status.state = SOEMMasterState::INITIALIZED;
            status_queue.push(status);
        }
    };

    void SOEMMaster::cyclic_task(std::chrono::microseconds cycle_time_us)
    {
        set_thread_RT();

        cyclic_master_running.store(true);
        auto time_next_loop = std::chrono::high_resolution_clock::now();

        SOEMMasterState state_flag = SOEMMasterState::RUNNING;
        status.state = SOEMMasterState::RUNNING;
        status.rx_error_count_consecutive = 0;
        status.tx_error_count_consecutive = 0;
        status.other_ec_error_count_consecutive = 0;

        const std::string task_logger_name = __logger_name + "_cyclic_task";

        while (!cyclic_master_terminate.load())
        {
            cycle_counter++; // int should be atomic

            // local state variable; commit to master status on end of cycle
            state_flag = SOEMMasterState::RUNNING;

            // get new output process data on request
            if (RxPDO_transfer.load())
            {
                std::copy(RxPDO_working.begin(), RxPDO_working.end(), RxPDO_IOmap.begin());
                RxPDO_transfer.store(false);
                RxPDO_transfer.notify_one();
            }

            // read/write process data
            if (ecx_send_overlap_processdata_group(&ecx_context, 0) <= 0)
            {
                if (status.tx_error_count_consecutive == 0)
                {
                    // no particularly good idea to log from realtime thread, but this is not the happy path anyways
                    // RCLCPP_WARN(rclcpp::get_logger(task_logger_name), "Sending process data failed!");
                }

                status.tx_error_count_consecutive++;
                status.tx_error_count++;
                status.error_count++;

                state_flag = SOEMMasterState::ERROR;
            }
            else
            {
                if (status.tx_error_count_consecutive != 0)
                {
                    // no particularly good idea to log from realtime thread, but this is not the happy path anyways
                    // RCLCPP_INFO(rclcpp::get_logger(task_logger_name), "Resumed sending process data from failure state!");
                }

                status.tx_error_count_consecutive = 0;
            }

            if (ecx_receive_processdata(&ecx_context, timeout_process_data.count()) <= 0)
            {
                if (status.rx_error_count_consecutive == 0)
                {
                    // no particularly good idea to log from realtime thread, but this is not the happy path anyways
                    // RCLCPP_WARN(rclcpp::get_logger(task_logger_name), "Receiving process data failed!");
                }

                status.rx_error_count_consecutive++;
                status.rx_error_count++;
                status.error_count++;

                state_flag = SOEMMasterState::ERROR;
            }
            else
            {
                if (status.rx_error_count_consecutive != 0)
                {
                    // no particularly good idea to log from realtime thread, but this is not the happy path anyways
                    // RCLCPP_INFO(rclcpp::get_logger(task_logger_name), "Resumed receiving process data from failure state!");
                }

                status.rx_error_count_consecutive = 0;
            }

            // make input data available on request
            if (TxPDO_transfer.load())
            {
                std::copy(TxPDO_IOmap.begin(), TxPDO_IOmap.end(), TxPDO_working.begin());
                TxPDO_transfer.store(false);
                TxPDO_transfer.notify_one();
            }

            // send all mailboxes
            for (auto &&slave_da : _slaves_data_access)
            {
                if (slave_da.send_mbx_request.load())
                {
                    // load data to send buffer; SMbx is buffer view to EC_SMbx
                    ec_clearmbx(&EC_SMbx);
                    std::copy(slave_da.SMbx.begin(), slave_da.SMbx.end(), SMbx.begin());

                    if (0 < ecx_mbxsend(&ecx_context, slave_da.slave_info.position, &EC_SMbx, timeout_mbx_send.count()))
                    {
                        // success; reset request
                        slave_da.send_mbx_request.store(false);
                        slave_da.send_mbx_request.notify_one();
                    }
                }
            }

            // receive all mailboxes
            for (auto &&slave_da : _slaves_data_access)
            {
                if (slave_da.receive_mbx.load() && !slave_da.receive_mbx_has_unread.load())
                {
                    ec_clearmbx(&EC_RMbx);
                    if (0 < ecx_mbxreceive(&ecx_context, slave_da.slave_info.position, &EC_RMbx, timeout_mbx_receive.count()))
                    {
                        // load data to slaves receive buffer; RMbx is buffer view to EC_RMbx
                        std::copy_n(RMbx.begin(), slave_da.RMbx.size(), slave_da.RMbx.begin());

                        slave_da.receive_mbx_has_unread.store(true);
                        slave_da.receive_mbx_has_unread.notify_one();
                    }
                }
            }

            // check for EtherCAT errors in SOEM
            if (ecx_iserror(&ecx_context))
            {
                // count errors and clear list
                size_t count = 0;
                ec_errort _;
                while (ecx_poperror(&ecx_context, &_))
                {
                    // TODO(bitmeal): how to handle informative logging
                    count++;
                };

                if (status.other_ec_error_count_consecutive == 0)
                {
                    // TODO(bitmeal): how to handle informative logging

                    // no particularly good idea to log from realtime thread, but this is not the happy path anyways
                    // RCLCPP_WARN(rclcpp::get_logger(task_logger_name), "EtherCAT bus reported errors");
                }

                status.other_ec_error_count_consecutive += count;
                status.error_count += count;

                state_flag = SOEMMasterState::ERROR;
            }
            else
            {
                if (status.other_ec_error_count_consecutive != 0)
                {
                    // no particularly good idea to log from realtime thread, but this is not the happy path anyways
                    // RCLCPP_INFO(rclcpp::get_logger(task_logger_name), "Resumed to a state where the bus reports no errors!");
                }

                status.other_ec_error_count_consecutive = 0;
            }

            // commit state
            status.state = state_flag;

            // prepare next cycle
            time_next_loop += cycle_time_us;
            auto time_now = std::chrono::high_resolution_clock::now();
            if (time_now > time_next_loop)
            {
                status.missed_cycle_deadline_count++;

                time_next_loop = time_now;
            }

            // push state to queue if space available
            status_queue.push(status);

            // sleep until next cycle
            std::this_thread::sleep_until(time_next_loop);
        }

        cyclic_master_running.store(false);
        status.state = SOEMMasterState::IDLE;
        status_queue.push(status);
    };

    // run cyclic master realtime task with cycle time
    void SOEMMaster::run(std::chrono::microseconds cycle_time_us)
    {
        bus_up_OP();
        ec_log_slaves();

        cycle_counter = 0;
        cyclic_master_terminate.store(false);

        cyclic_master = std::thread{&SOEMMaster::cyclic_task, this, cycle_time_us};
    };

    // transfer RxPDO working set to realtime context and send to bus
    void SOEMMaster::transfer_RxPDO()
    {
        // RxPDO_transfer_queue.push(RxPDO_working);

        if (cyclic_master_running.load() && !cyclic_master_terminate.load())
        {
            RxPDO_transfer.store(true);
            RxPDO_transfer.wait(true);
        }
    };
    // transfer latest data from bus to TxPDO working set
    void SOEMMaster::transfer_TxPDO()
    {
        // TxPDO_transfer_queue.consume_all([&](std::vector<std::byte> TxPDO)
        //                                  { std::copy(TxPDO.begin(), TxPDO.end(), TxPDO_working.begin()); });

        if (cyclic_master_running.load() && !cyclic_master_terminate.load())
        {
            TxPDO_transfer.store(true);
            TxPDO_transfer.wait(true);
        }
    };

    void SOEMMaster::stop()
    {
        if (cyclic_master.joinable())
        {
            cyclic_master_terminate.store(true);
            cyclic_master.join();
        }

        cyclic_master_running.store(false);
        cyclic_master_terminate.store(false);

        if (is_init)
        {
            bus_down_SAFE_OP();
        }
    };

    void SOEMMaster::slave_attach_SDO_setup_hook(const SOEMEcSlaveInfo &slave, std::function<void(soem_driver::SDOwrite_t)> hook_fn)
    {
        SDO_setup_hook_store[slave.position] = hook_fn;
        ec_slave[slave.position].PO2SOconfigx = fnptr_call_SOEMMaster_member<&SOEMMaster::call_slave_SDO_setup_hook>;
    };

    // get buffer to working set of slaves RxPDO
    // call after calling start_bus() only!
    const soem_driver::buffer SOEMMaster::getRxPDO(SOEMEcSlaveInfo slave)
    {
        size_t slave_outputs_offset = ec_slave[slave.position].outputs - ec_group[0].outputs;
        return {RxPDO_working.data() + slave_outputs_offset, ec_slave[slave.position].Obytes};
    };

    // get buffer to working set of slaves TxPDO
    // call after calling start_bus() only!
    const soem_driver::buffer SOEMMaster::getTxPDO(SOEMEcSlaveInfo slave)
    {
        size_t slave_inputs_offset = ec_slave[slave.position].inputs - ec_group[0].inputs;
        return {TxPDO_working.data() + slave_inputs_offset, ec_slave[slave.position].Ibytes};
    };

    const soem_driver::buffer SOEMMaster::getSMbx(SOEMEcSlaveInfo slave)
    {
        auto holding_buffer = SMbx_working[slave.position - 1];
        return {holding_buffer.data(), holding_buffer.size()};
    };
    const soem_driver::buffer SOEMMaster::getRMbx(SOEMEcSlaveInfo slave)
    {
        auto holding_buffer = RMbx_working[slave.position - 1];
        return {holding_buffer.data(), holding_buffer.size()};
    };

    // setup function passed to SOEM
    int SOEMMaster::call_slave_SDO_setup_hook(uint16_t slave_position)
    {
        int wkc = 0;
        auto setup_fn_it = SDO_setup_hook_store.find(slave_position);
        if (setup_fn_it != SDO_setup_hook_store.end())
        {
            auto sdo_write_fn = [&](uint16_t index, uint8_t sub_index, soem_driver::buffer data, bool complete_access)
            {
                // wkc += SDOwrite_slave(slave, index, sub_index, data, complete_access);
                wkc += ecx_SDOwrite(&ecx_context, slave_position, index, sub_index, complete_access, data.size_bytes(), data.data(), EC_TIMEOUTSAFE);
            };

            // call slaves setup hook
            setup_fn_it->second(sdo_write_fn);
        }
        return wkc;
    };

    void SOEMMaster::ec_log_slaves()
    {
        for (auto slave_idx : std::ranges::views::iota(1, ec_slavecount + 1))
        {
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "Slave %3i [%-7s] %s (%s)", slave_idx, ec_state_to_string(ec_slave[slave_idx].state).c_str(), ec_slave[slave_idx].name, ec_ALstatuscode2string(ec_slave[slave_idx].ALstatuscode));
        }
    };

    void SOEMMaster::set_thread_RT()
    {
        // set nice value first, in case RT scheduling fails
        // MIN_NICE + 1
        // if (setpriority(PRIO_PROCESS, getpid(), -19))
        if (setpriority(PRIO_PROCESS, 0, -19))
        {
            RCLCPP_WARN(rclcpp::get_logger(__logger_name), "failed to set SEOM master thread high priority");
        }

        // set RT scheduling
        // IRQs have max/2
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO) / 2 - 1;
        if (sched_setscheduler(0, SCHED_FIFO, &param))
        {
            RCLCPP_WARN(rclcpp::get_logger(__logger_name), "failed to set SEOM master thread RT SCHED_FIFO");
            return;
        }

        // lock memory and pre-fault stack
        if (mlockall(MCL_CURRENT | MCL_FUTURE))
        {
            RCLCPP_WARN(rclcpp::get_logger(__logger_name), "failed to lock memory fo SEOM master thread");
            return;
        }

        // TODO(bitmeal): optimized out? optimization?
        constexpr int MAX_SAFE_STACK = 8 * 1024;
        unsigned char dummy[MAX_SAFE_STACK];
        memset(dummy, 0, MAX_SAFE_STACK);
    };

    int SOEMMaster::get_cycle_counter()
    {
        return cycle_counter;
    };

} // namespace soem_master