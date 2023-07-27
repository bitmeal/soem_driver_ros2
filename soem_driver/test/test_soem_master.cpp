#include <cstdlib>

#include "soem_driver/soem_master.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace soem_master;


const std::string __logger_name{"test_soem_master"};

int main(int argc, char const *argv[])
{
    if(argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "give network interface to use as positional parameter");
        return EXIT_FAILURE;
    }

    SOEMMaster master;
    master.init(argv[1]);

    return EXIT_SUCCESS;
}
