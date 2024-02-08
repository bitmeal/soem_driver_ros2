#ifndef SOEM_DRIVER__ALIAS_INTERFACE_FACTORY_HPP_
#define SOEM_DRIVER__ALIAS_INTERFACE_FACTORY_HPP_

#include <string>
#include <type_traits>

#include "hardware_interface/handle.hpp"

namespace soem_driver
{
    class AliasInterfaceFactory : public hardware_interface::ReadOnlyHandle
    {
    public:
        AliasInterfaceFactory() = delete;
        AliasInterfaceFactory(const AliasInterfaceFactory &other) = delete;
        AliasInterfaceFactory(AliasInterfaceFactory &&other) = default;

        double get_value() = delete;

        AliasInterfaceFactory(const hardware_interface::ReadOnlyHandle &target) : hardware_interface::ReadOnlyHandle(target){};

        template <typename T, std::enable_if_t<std::is_base_of_v<ReadOnlyHandle, T>, bool> = true>
        T makeInterface(const std::string &prefix, const std::string &name)
        {
            // return std::move(T{prefix, name, value_ptr_});
            return T{prefix, name, value_ptr_};
        }
    };
} // namespace soem_driver

#endif // SOEM_DRIVER__ALIAS_INTERFACE_FACTORY_HPP_