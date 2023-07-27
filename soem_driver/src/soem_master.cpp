#include "soem_driver/soem_master.hpp"

#include <exception>
#include <ranges>

#include "rclcpp/rclcpp.hpp"

namespace soem_master
{
    extern "C"
    {
// #include "soem/ethercat.h"
#include "soem/ethercattype.h"
#include "soem/nicdrv.h"
#include "soem/ethercatbase.h"
#include "soem/ethercatmain.h"
#include "soem/ethercatconfig.h"
#include "soem/ethercatcoe.h"
#include "soem/ethercatdc.h"
#include "soem/ethercatprint.h"
    }

    SOEMMaster::SOEMMaster()
        : slaves(_slaves), __logger_name("soem_master"){};
    SOEMMaster::~SOEMMaster(){
              ec_close();
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "closed EtherCAT communication");
    };

    // initialize interface and scan bus for slaves
    void SOEMMaster::init(const std::string &interface)
    {
        if (ec_init(interface.c_str()))
        {
            RCLCPP_INFO(rclcpp::get_logger(__logger_name), "initialized EtherCAT communication on interface %s", interface.c_str());

            for (auto slave_idx : std::ranges::views::iota(0, ec_slavecount))
            {
                RCLCPP_INFO(rclcpp::get_logger(__logger_name), "slave %i: %s", slave_idx, ec_slave[slave_idx].name);
            }
        }
        else
        {
            throw std::runtime_error("could not initialize EtherCAT communication on interface " + interface);
        }
    };
    // get all devices to OP state, calling attached SDO setup hooks on transition from PO to SO
    void SOEMMaster::start_bus(){};

    // run cyclic master realtime task with cycle time
    void SOEMMaster::run(int cycle_time){};
    void SOEMMaster::stop(){};

    void SOEMMaster::slave_attach_SDO_setup_hook(const SOEMEcSlaveInfo &slave, std::function<void(soem_driver::SDOwrite_t)> hook_fn){};
    soem_driver::buffer SOEMMaster::getRxPDO(SOEMEcSlaveInfo){};
    const soem_driver::buffer SOEMMaster::getTxPDO(SOEMEcSlaveInfo){};

    void SOEMMaster::SDOwrite(uint16_t index, uint8_t sub_index, soem_driver::buffer data){};

} // namespace soem_master