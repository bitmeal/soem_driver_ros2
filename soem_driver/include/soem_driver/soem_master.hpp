#ifndef SOEM_DRIVER__SOEM_MASTER_HPP_
#define SOEM_DRIVER__SOEM_MASTER_HPP_

#include <memory>
#include <vector>
#include <deque>
#include <map>
#include <thread>
#include <chrono>
#include <atomic>
// #include <boost/lockfree/spsc_queue.hpp>

#include "soem_driver_common/soem_driver_common.hpp"
extern "C"
{
#include "soem/ethercat.h"
}

namespace soem_master
{
    inline constexpr size_t cacheline_width = 64;

    struct SOEMEcSlaveInfo
    {
        uint64_t vendor_id;
        uint64_t product_code;
        uint64_t revision_number;

        int position;
        int alias;

        std::string name;

        // PDO sizes as minimum number of bytes to hold data; slave with one one-bit-I/O has one byte
        // output data size
        size_t RxPDO_size;
        // input data size
        size_t TxPDO_size;
    };

    struct SOEMEcSlaveDataAccess
    {
        SOEMEcSlaveDataAccess(
            SOEMEcSlaveInfo &slave_info,
            soem_driver::buffer RxPDO,
            const soem_driver::buffer TxPDO,
            soem_driver::buffer SMbx,
            const soem_driver::buffer RMbx
            ) : slave_info(slave_info),
                RxPDO(RxPDO),
                TxPDO(TxPDO),
                SMbx(SMbx),
                RMbx(RMbx),
                send_mbx_request(false),
                receive_mbx(false),
                receive_mbx_has_unread(false){};

        SOEMEcSlaveInfo &slave_info;

        // output data
        const soem_driver::buffer RxPDO;
        // input data
        const soem_driver::buffer TxPDO;
        // send mailbox
        const soem_driver::buffer SMbx;
        // receive mailbox
        const soem_driver::buffer RMbx;

        // requests send of mailbox buffer SMbx, resets after send
        alignas(cacheline_width) std::atomic_bool send_mbx_request;

        // mailbox will be queried continuously if true, and no unread data in buffer
        alignas(cacheline_width) std::atomic_bool receive_mbx;
        // indicates unread mailbox message in buffer, reset after read to get new messages
        alignas(cacheline_width) std::atomic_bool receive_mbx_has_unread;
    };

    const std::string ec_state_to_string(/* ec_state */ uint16_t state);

    class SOEMMaster
    {
    public:
        SOEMMaster();
        ~SOEMMaster();

        // EC configuration
        std::chrono::microseconds timeout_process_data; // = EC_TIMEOUTRET
        std::chrono::microseconds timeout_mbx_send;     // = EC_TIMEOUTTXM
        std::chrono::microseconds timeout_mbx_receive;  // = EC_TIMEOUTRXM

        std::string __logger_name;
        const std::deque<SOEMEcSlaveInfo> &slaves;
        // TODO(bitmeal): fix constnes
        // const STL containers propagate const to elements; span does not
        // only populated after calling start_bus()
        std::deque<SOEMEcSlaveDataAccess> &slaves_data_access;

        // initialize interface and scan bus for slaves
        void init(const std::string &interface);
        // get all devices to OP state, calling attached SDO setup hooks on transition from PO to SO
        void start_bus(size_t IOmap_size = 4096);

        // run cyclic master realtime task with cycle time
        void run(std::chrono::microseconds cycle_time_us);

        // transfer RxPDO working set to realtime context and send to bus
        void transfer_RxPDO();
        // transfer latest data from bus to TxPDO working set
        void transfer_TxPDO();

        void stop();

        int get_cycle_counter();

        void slave_attach_SDO_setup_hook(const SOEMEcSlaveInfo &slave, std::function<void(soem_driver::SDOwrite_t)> hook_fn);

    private:
        std::map<uint16_t, std::function<void(soem_driver::SDOwrite_t)>> SDO_setup_hook_store;

        // PDOs
        std::vector<std::byte> IOmap;

        // RxPDO / Output
        soem_driver::buffer RxPDO_IOmap;
        std::vector<std::byte> RxPDO_working;
        alignas(cacheline_width) std::atomic_bool RxPDO_transfer;
        // boost::lockfree::spsc_queue<std::vector<std::byte>> RxPDO_transfer_queue;

        // TxPDO / Input
        soem_driver::buffer TxPDO_IOmap;
        std::vector<std::byte> TxPDO_working;
        alignas(cacheline_width) std::atomic_bool TxPDO_transfer;
        // boost::lockfree::spsc_queue<std::vector<std::byte>> TxPDO_transfer_queue;

        // MBX
        // send MBX / slave IN/RX
        ec_mbxbuft EC_SMbx;
        soem_driver::buffer SMbx;
        std::vector<std::vector<std::byte>> SMbx_working;

        // receive MBX / slave OUT/TX
        ec_mbxbuft EC_RMbx;
        soem_driver::buffer RMbx;
        std::vector<std::vector<std::byte>> RMbx_working;
        
        // get buffer to working set of slaves RxPDO & RxPDO
        // call after calling start_bus() only!
        // private now: use slave data access structures for slave DA!
        const soem_driver::buffer getRxPDO(SOEMEcSlaveInfo slave);
        const soem_driver::buffer getTxPDO(SOEMEcSlaveInfo slave);
        const soem_driver::buffer getSMbx(SOEMEcSlaveInfo slave);
        const soem_driver::buffer getRMbx(SOEMEcSlaveInfo slave);
        
        // slave information/access structures
        std::deque<SOEMEcSlaveInfo> _slaves;
        std::deque<SOEMEcSlaveDataAccess> _slaves_data_access;

        void ec_log_slaves();

        // TODO(bitmeal): static function does not allow multiple master instances and will lead to colissions!
        int call_slave_SDO_setup_hook(uint16_t slave_position);

        ////////////////////////////
        // SOEM context
        // from ethercatmain.c
        // TODO(bitmeal): wrap in context owning structure

        ec_slavet ec_slave[EC_MAXSLAVE];
        int ec_slavecount;
        ec_groupt ec_group[EC_MAXGROUP];

        uint8_t ec_esibuf[EC_MAXEEPBUF];
        uint32_t ec_esimap[EC_MAXEEPBITMAP];
        ec_eringt ec_elist;
        ec_idxstackT ec_idxstack;

        ec_SMcommtypet ec_SMcommtype[EC_MAX_MAPT];
        ec_PDOassignt ec_PDOassign[EC_MAX_MAPT];
        ec_PDOdesct ec_PDOdesc[EC_MAX_MAPT];

        ec_eepromSMt ec_SM;
        ec_eepromFMMUt ec_FMMU;
        boolean EcatError = FALSE;

        int64_t ec_DCtime;

        ecx_portt ecx_port;
        ecx_redportt ecx_redport;

        ecx_contextt ecx_context;
        // ecx_contextt ecx_context = {
        //     &ecx_port,         // .port          =
        //     &ec_slave[0],      // .slavelist     =
        //     &ec_slavecount,    // .slavecount    =
        //     EC_MAXSLAVE,       // .maxslave      =
        //     &ec_group[0],      // .grouplist     =
        //     EC_MAXGROUP,       // .maxgroup      =
        //     &ec_esibuf[0],     // .esibuf        =
        //     &ec_esimap[0],     // .esimap        =
        //     0,                 // .esislave      =
        //     &ec_elist,         // .elist         =
        //     &ec_idxstack,      // .idxstack      =
        //     &EcatError,        // .ecaterror     =
        //     0,                 // .DCtO          =
        //     0,                 // .DCl           =
        //     &ec_DCtime,        // .DCtime        =
        //     &ec_SMcommtype[0], // .SMcommtype    =
        //     &ec_PDOassign[0],  // .PDOassign     =
        //     &ec_PDOdesc[0],    // .PDOdesc       =
        //     &ec_SM,            // .eepSM         =
        //     &ec_FMMU,          // .eepFMMU       =
        //     NULL,              // .FOEhook()
        //     NULL,              // .EOEhook()
        //     0                  // .manualstatechange
        // };
        ////////////////////////////

        bool is_init;

        alignas(cacheline_width) std::atomic_int cycle_counter;
        alignas(cacheline_width) std::thread cyclic_master;
        alignas(cacheline_width) std::atomic_bool cyclic_master_terminate;
        alignas(cacheline_width) std::atomic_bool cyclic_master_running;

        // boost::lockfree::spsc_queue<std::string> cyclic_master_logger_queue;

        void set_thread_RT();

        void bus_up_OP();
        void bus_down_SAFE_OP();

        void cyclic_task(std::chrono::microseconds cycle_time_us);
    };
} // namespace soem_master

#endif // SOEM_DRIVER__SOEM_MASTER_HPP_