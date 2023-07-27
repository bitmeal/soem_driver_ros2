
#ifndef SOEM_DRIVER_SLAVE_INTERFACE__SOEM_SLAVE_HPP_
#define SOEM_DRIVER_SLAVE_INTERFACE__SOEM_SLAVE_HPP_

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <algorithm>

#include "hardware_interface/system_interface.hpp"

#include "soem_driver_common/soem_driver_common.hpp"

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
        SOEMDriverSlave() : RxPDO(_RxPDO),
                      TxPDO(_TxPDO),
                      mbx_send_pending(_mbx_send_pending){};

    private:
        soem_driver::buffer _RxPDO;
        soem_driver::buffer _TxPDO;

        bool _mbx_send_pending = false;
        std::function<void(const soem_driver::buffer)> _mbx_callback;
        soem_driver::buffer _TxMbx;

        void _dispatch_Mbx_callback(soem_driver::buffer msg)
        {
            if (_mbx_callback)
            {
                // call the user provided callback
                _mbx_callback(msg);

                // if new send is pending, a new send has been scheduled from within callback
                // if not pending, clear callback
                if (!_mbx_send_pending)
                {
                    _mbx_callback = nullptr;
                }
            }
        };

    protected:
        // PDO buffers will be assigned by drivers; buffer itself is const, data in buffer is non-const
        const soem_driver::buffer &RxPDO;
        const soem_driver::buffer &TxPDO;

        const bool &mbx_send_pending;
        bool schedule_Mbx_send(const soem_driver::buffer msg, std::function<void(const soem_driver::buffer)> callback)
        {
            if (msg.size() > _TxMbx.size())
            {
                // TODO(bitmeal): log error?
                return false;
            }

            std::copy(msg.begin(), msg.end(), _TxMbx.begin());
            _mbx_send_pending = true;
            _mbx_callback = callback;

            return true;
        };

    public:
        virtual bool init(std::unordered_map<std::string, std::string> parameters) { return true; };

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() = 0;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() = 0;

        virtual void configure(
            uint64_t vendor_id,
            uint64_t product_code,
            uint64_t revision_number,
            std::unordered_map<std::string, std::string> parameters) = 0;

        virtual void setup_SDO_hook(soem_driver::SDOwrite_t SDOwrite){};

        virtual hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) = 0;
        virtual hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) = 0;
    };
} // namespace soem_driver_slave_interface

#endif // SOEM_DRIVER_SLAVE_INTERFACE__SOEM_SLAVE_HPP_
