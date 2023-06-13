
#ifndef SOEM_SLAVE_INTERFACE__SOEM_SLAVE_HPP_
#define SOEM_SLAVE_INTERFACE__SOEM_SLAVE_HPP_

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <algorithm>
#include <span>
#include <cstddef>

#include "hardware_interface/system_interface.hpp"

// namespace soem_slave_interface
// {
//     class SOEMSlave;
// }
// #include "soem_driver/soem_driver.hpp"

namespace soem_driver
{
    class SOEMDriver;
    typedef std::span<std::byte> buffer;
} // namespace soem_driver

// class soem_driver::SOEMDriver;
// typedef std::span<std::byte> soem_driver::buffer;

namespace soem_slave_interface
{
    template <typename I>
    std::vector<I> list_initialize_non_copyable_interface(const std::initializer_list<std::tuple<std::string, std::string, double *>>& params)
    {
        std::vector<I> target;
        std::transform(params.begin(), params.end(), std::back_inserter(target),
                       [](auto pack)
                       {
                           return std::make_from_tuple<I>(pack);
                       });
        return target;
    };

    class SOEMSlave
    {
        // allow private member access from driver to setup buffers
        friend class soem_driver::SOEMDriver;

    public:
        SOEMSlave() : RxPDO(_RxPDO),
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

        typedef std::function<void(uint16_t index, uint8_t sub_index, soem_driver::buffer data)> SDOwrite_t;
        virtual void setup_SDO_hook(SDOwrite_t SDOwrite){};

        virtual hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) = 0;
        virtual hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) = 0;
    };
} // namespace soem_slave_interface

#endif // SOEM_SLAVE_INTERFACE__SOEM_SLAVE_HPP_
