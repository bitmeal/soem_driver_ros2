#ifndef SOEM_DRIVER__SOEM_MASTER_HELPERS_ROS_HPP_
#define SOEM_DRIVER__SOEM_MASTER_HELPERS_ROS_HPP_

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

#include "soem_driver/soem_master.hpp"

namespace soem_master
{
    diagnostic_msgs::msg::DiagnosticStatus build_master_status_diagnostics_ros(const SOEMMaster::SOEMMasterStatus status)
    {
            diagnostic_updater::DiagnosticStatusWrapper builder{};

            builder.add("state", SOEMMaster::master_state_to_string(status.state));
            builder.add("error_count", status.error_count);
            builder.add("rx_error_count", status.rx_error_count);
            builder.add("tx_error_count", status.tx_error_count);
            builder.add("missed_cycle_deadline_count", status.missed_cycle_deadline_count);
            // builder.add("", status.rx_error_count_consecutive);
            // builder.add("", status.tx_error_count_consecutive);
            // builder.add("", status.other_ec_error_count_consecutive);
            
            return builder;
    };

} // namespace soem_master

#endif // SOEM_DRIVER__SOEM_MASTER_HELPERS_ROS_HPP_