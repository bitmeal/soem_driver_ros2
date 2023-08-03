#include <cstdlib>

#include "soem_driver/soem_master.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace soem_master;

////////////////////////////////////
#include <ctype.h>
#include <stdio.h>

void hexdump(void *ptr, int buflen) {
  unsigned char *buf = (unsigned char*)ptr;
  int i, j;
  for (i=0; i<buflen; i+=16) {
    printf("%06x: ", i);
    for (j=0; j<16; j++) 
      if (i+j < buflen)
        printf("%02x ", buf[i+j]);
      else
        printf("   ");
    printf(" ");
    for (j=0; j<16; j++) 
      if (i+j < buflen)
        printf("%c", isprint(buf[i+j]) ? buf[i+j] : '.');
    printf("\n");
  }
}
////////////////////////////////////


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

    master.slave_attach_SDO_setup_hook(master.slaves[0], [](auto SDOwrite_fn){
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "hello from PO2SO hook for slave 0");
    });

    master.slave_attach_SDO_setup_hook(master.slaves[5], [](auto SDOwrite_fn){
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "hello from PO2SO hook for slave 5");
    });

    master.start_bus();

    master.run(std::chrono::microseconds(1000));

    auto TxPDO = master.getTxPDO(master.slaves[1]);
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(rclcpp::get_logger(__logger_name), "cycle count: %d", master.get_cycle_counter());

        master.transfer_TxPDO();
        // printf("cycle count: %d\n", master.get_cycle_counter());
        hexdump(TxPDO.data(), TxPDO.size());
    }

    master.stop();
    RCLCPP_INFO(rclcpp::get_logger(__logger_name), "cycle counter: %d", master.get_cycle_counter());

    return EXIT_SUCCESS;
}
