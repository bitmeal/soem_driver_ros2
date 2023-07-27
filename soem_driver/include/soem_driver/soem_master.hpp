#ifndef SOEM_DRIVER__SOEM_MASTER_HPP_
#define SOEM_DRIVER__SOEM_MASTER_HPP_

#include <memory>
#include <vector>
#include <map>

#include "soem_driver_common/soem_driver_common.hpp"

namespace soem_master
{
    struct SOEMEcSlaveInfo
    {
        uint64_t vendor_id;
        uint64_t product_code;
        uint64_t revision_number;

        int position;
        int alias;

        std::string name;

        size_t RxPDO_size;
        size_t TxPDO_size;
    };

    class SOEMMaster
    {
    public:
        SOEMMaster();
        ~SOEMMaster();

        std::string __logger_name;
        const std::vector<SOEMEcSlaveInfo> &slaves;

        // initialize interface and scan bus for slaves
        void init(const std::string &interface);
        // get all devices to OP state, calling attached SDO setup hooks on transition from PO to SO
        void start_bus();

        // run cyclic master realtime task with cycle time
        void run(int cycle_time);
        void stop();

        void slave_attach_SDO_setup_hook(const SOEMEcSlaveInfo &slave, std::function<void(soem_driver::SDOwrite_t)> hook_fn);
        soem_driver::buffer getRxPDO(SOEMEcSlaveInfo);
        const soem_driver::buffer getTxPDO(SOEMEcSlaveInfo);

    private:
        std::vector<std::byte> IOmap;

        // RxPDO / Output
        soem_driver::buffer RxPDO;
        // TxPDO / Input
        soem_driver::buffer TxPDO;

        std::vector<SOEMEcSlaveInfo> _slaves;

        // soem_driver::SDOwrite_t
        void SDOwrite(uint16_t index, uint8_t sub_index, soem_driver::buffer data);
    };
} // namespace soem_master

#endif // SOEM_DRIVER__SOEM_MASTER_HPP_