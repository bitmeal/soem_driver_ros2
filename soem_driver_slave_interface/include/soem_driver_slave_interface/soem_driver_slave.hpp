
#ifndef SOEM_DRIVER_SLAVE_INTERFACE__SOEM_SLAVE_HPP_
#define SOEM_DRIVER_SLAVE_INTERFACE__SOEM_SLAVE_HPP_

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <functional>
#include <algorithm>
#include <tuple>

#include "hardware_interface/system_interface.hpp"

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

#include "soem_driver_common/soem_driver_common.hpp"
#include <boost/lockfree/spsc_queue.hpp>

namespace soem_driver
{
    class SOEMDriver;
} // namespace soem_driver

namespace soem_driver_slave_interface
{
    class SOEMDriverSlave
    {
        // allow private member access from driver to setup buffers
        friend class soem_driver::SOEMDriver;

    public:
        SOEMDriverSlave()
            : _position_ec_bus(0),
              mbx_send_queue({8}),
              mbx_receive_queue({8}),
              RxPDO(_RxPDO),
              TxPDO(_TxPDO)
              {};

    private:
        // will be assigned by driver, access using RxPDO and TxPDO references
        soem_driver::buffer _RxPDO;
        soem_driver::buffer _TxPDO;

        int _position_ec_bus;

        boost::lockfree::spsc_queue<std::tuple<std::vector<std::byte>, bool, bool>> mbx_send_queue;
        boost::lockfree::spsc_queue<std::vector<std::byte>> mbx_receive_queue;

    protected:
        // PDO buffers will be assigned by drivers; buffer itself is const, data in buffer is non-const
        // access buffers in read() and write() only!
        // output
        const soem_driver::buffer &RxPDO;
        // input
        const soem_driver::buffer &TxPDO;


        boost::lockfree::spsc_queue<diagnostic_msgs::msg::DiagnosticStatus, boost::lockfree::capacity<1>> diagnostics_status_queue;

        // enqueue messages to be sent to slave mailbox
        // replies will only be fetched if fetch_mailbox = true
        // method takes care of owning the data sent in non owning buffer
        bool mbx_enqueue_send(const soem_driver::buffer msg, bool read_response = true, bool read_mbx_raw = false)
        {
            return mbx_send_queue.push({{msg.begin(), msg.end()}, read_response, read_mbx_raw});
        };

        // // check mailbox for new mail every cycle
        // bool fetch_mailbox;

        // consume all received mailbox messages with callback function
        // returns number of consumed messages
        // consuming incoming messages is responsibility of the slave
        size_t mbx_consume_incoming(std::function<void(const soem_driver::buffer msg)> callback)
        {
            return mbx_receive_queue.consume_all(
                [&](auto mbx_in)
                {
                    callback({mbx_in.begin(), mbx_in.end()});
                });
        };

    public:
        virtual bool init(std::unordered_map<std::string, std::string> /* parameters */) { return true; };

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() = 0;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() = 0;

        // return success value of configuration
        virtual bool configure(
            uint64_t vendor_id,
            uint64_t product_code,
            uint64_t revision_number,
            std::unordered_map<std::string, std::string> parameters) = 0;

        virtual void setup_SDO_hook(soem_driver::SDOwrite_t /* SDOwrite */){};

        virtual hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> & /*start_interfaces*/,
            const std::vector<std::string> & /*stop_interfaces*/)
        {
            return hardware_interface::return_type::OK;
        };

        virtual hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string> & /*start_interfaces*/,
            const std::vector<std::string> & /*stop_interfaces*/)
        {
            return hardware_interface::return_type::OK;
        };

        virtual hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) = 0;
        virtual hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) = 0;
    };
} // namespace soem_driver_slave_interface

#endif // SOEM_DRIVER_SLAVE_INTERFACE__SOEM_SLAVE_HPP_
