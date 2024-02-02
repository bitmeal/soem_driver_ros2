
#ifndef SOEM_DRIVER_SLAVE_INTERFACE__SOEM_SLAVE_HPP_
#define SOEM_DRIVER_SLAVE_INTERFACE__SOEM_SLAVE_HPP_

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <functional>
#include <algorithm>

#include "hardware_interface/system_interface.hpp"

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
              mbx_send_queue({}),
              mbx_receive_queue({}),
              RxPDO(_RxPDO),
              TxPDO(_TxPDO),
              fetch_mailbox(false)
              //   mbx_send_pending(_mbx_send_pending)
              {};

    private:
        // will be assigned by driver, access using RxPDO and TxPDO references
        soem_driver::buffer _RxPDO;
        soem_driver::buffer _TxPDO;

        int _position_ec_bus;

        boost::lockfree::spsc_queue<std::vector<std::byte>> mbx_send_queue;
        boost::lockfree::spsc_queue<std::vector<std::byte>> mbx_receive_queue;
        // std::deque<std::vector<std::byte>> mbx_send_queue;
        // std::deque<std::vector<std::byte>> mbx_receive_queue;

        // bool _mbx_send_pending = false;
        // std::function<void(const soem_driver::buffer)> _mbx_callback;
        // soem_driver::buffer _TxMbx; // send

        // void _dispatch_Mbx_callback(soem_driver::buffer msg)
        // {
        //     if (_mbx_callback)
        //     {
        //         // call the user provided callback
        //         _mbx_callback(msg);

        //         // if new send is pending, a new send has been scheduled from within callback
        //         // if not pending, clear callback
        //         if (!_mbx_send_pending)
        //         {
        //             _mbx_callback = nullptr;
        //         }
        //     }
        // };

    protected:
        // PDO buffers will be assigned by drivers; buffer itself is const, data in buffer is non-const
        // access buffers in read() and write() only!
        // output
        const soem_driver::buffer &RxPDO;
        // input
        const soem_driver::buffer &TxPDO;

        // enqueue messages to be sent to slave mailbox
        // replies will only be fetched if fetch_mailbox = true
        bool mbx_enqueue_send(const soem_driver::buffer msg)
        {
            // std::vector<std::byte> holder(msg.begin(), msg.end());
            // mbx_send_queue.push(holder);

            return mbx_send_queue.push({msg.begin(), msg.end()});
        };

        // check mailbox for new mail every cycle
        bool fetch_mailbox;
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

        // const bool &mbx_send_pending;
        // bool schedule_Mbx_send(const soem_driver::buffer msg, std::function<void(const soem_driver::buffer)> callback)
        // {
        //     if (msg.size() > _TxMbx.size())
        //     {
        //         // TODO(bitmeal): log error?
        //         return false;
        //     }

        //     std::copy(msg.begin(), msg.end(), _TxMbx.begin());
        //     _mbx_send_pending = true;
        //     _mbx_callback = callback;

        //     return true;
        // };

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
