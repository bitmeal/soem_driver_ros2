#ifndef SOEM_DRIVER_COMMON__SOEM_DRIVER_COMMON_HPP_
#define SOEM_DRIVER_COMMON__SOEM_DRIVER_COMMON_HPP_

#include <vector>
#include <utility>
#include <span>
#include <cstddef>
#include <functional>
#include <algorithm>

namespace soem_driver
{
    typedef std::span<std::byte> buffer;

    typedef std::function<void(uint16_t index, uint8_t sub_index, soem_driver::buffer data)> SDOwrite_t;

    template <typename I>
    std::vector<I> list_initialize_non_copyable_interface(const std::initializer_list<std::tuple<std::string, std::string, double *>> &params)
    {
        std::vector<I> target;
        std::transform(params.begin(), params.end(), std::back_inserter(target),
                       [](auto pack)
                       {
                           return std::make_from_tuple<I>(pack);
                       });
        return target;
    };

} // namespace soem_driver

#endif // SOEM_DRIVER_COMMON__SOEM_DRIVER_COMMON_HPP_