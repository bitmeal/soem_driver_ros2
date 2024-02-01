#ifndef SOEM_MODULES_TRINAMIC__COMMON_HPP_
#define SOEM_MODULES_TRINAMIC__COMMON_HPP_

#include <cstdint>
#include <climits>
#include <algorithm>
#include <bit>
#include <array>

namespace soem_slave_modules
{
    namespace trinamic
    {
        // logic only implemented for 8 bit byte width
        static_assert(CHAR_BIT == 8);

        enum Endianness
        {
            BIG,
            LITTLE
        };

        constexpr Endianness get_endianness()
        {
            constexpr uint64_t token = 0x01;
            return std::bit_cast<std::array<std::byte, sizeof(token)>>(token)[0] == std::byte{0x01} ? Endianness::LITTLE : Endianness::BIG;
        }

        template <typename T>
        inline constexpr T swap_endianness(const T &data)
        {
            auto swap_data = std::bit_cast<std::array<std::byte, sizeof(T)>>(data);
            std::reverse(swap_data.begin(), swap_data.end());
            return std::move(std::bit_cast<T>(swap_data));
        }

        template <typename T>
        inline constexpr T swap_endianness_other_is_big(const T &data)
        {
            return get_endianness() == Endianness::LITTLE ? std::move(swap_endianness(data)) : data;
        }

        template <typename T>
        inline constexpr T swap_endianness_other_is_little(const T &data)
        {
            return get_endianness() == Endianness::BIG ? std::move(swap_endianness(data)) : data;
        }

        template <typename T>
        inline constexpr T to_big_endian(const T &data)
        {
            return swap_endianness_other_is_big(data);
        }

        template <typename T>
        inline constexpr T from_big_endian(const T &data)
        {
            return swap_endianness_other_is_big(data);
        }

        template <typename T>
        inline constexpr T to_little_endian(const T &data)
        {
            return swap_endianness_other_is_little(data);
        }

        template <typename T>
        inline constexpr T from_little_endian(const T &data)
        {
            return swap_endianness_other_is_little(data);
        }

    } // namespace trinamic
} // namespace soem_slave_modules

#endif // SOEM_MODULES_TRINAMIC__COMMON_HPP_
