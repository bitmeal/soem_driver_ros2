#include <cstdlib>

#include "soem_driver/soem_master.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace soem_master;

////////////////////////////////////
#include <ctype.h>
#include <stdio.h>

void hexdump(void *ptr, int buflen)
{
  unsigned char *buf = (unsigned char *)ptr;
  int i, j;
  for (i = 0; i < buflen; i += 16)
  {
    printf("%06x: ", i);
    for (j = 0; j < 16; j++)
      if (i + j < buflen)
        printf("%02x ", buf[i + j]);
      else
        printf("   ");
    printf(" ");
    for (j = 0; j < 16; j++)
      if (i + j < buflen)
        printf("%c", isprint(buf[i + j]) ? buf[i + j] : '.');
    printf("\n");
  }
}
////////////////////////////////////

const std::string __logger_name{"test_soem_master"};

int main(int argc, char const *argv[])
{
  if (argc != 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__logger_name), "give network interface to use as positional parameter");
    return EXIT_FAILURE;
  }

  // add unused master to check for setup hook resolution
  SOEMMaster dummy_master;
  
  SOEMMaster master;
  master.init(argv[1]);
  master.timeout_process_data = std::chrono::microseconds{250};
  master.timeout_mbx_send = std::chrono::microseconds{100};
  master.timeout_mbx_receive = std::chrono::microseconds{100};

  master.slave_attach_SDO_setup_hook(master.slaves[0], [](auto)
                                     { RCLCPP_INFO(rclcpp::get_logger(__logger_name), "hello from PO2SO hook for slave 0"); });

  master.slave_attach_SDO_setup_hook(master.slaves[5], [](auto)
                                     { RCLCPP_INFO(rclcpp::get_logger(__logger_name), "hello from PO2SO hook for slave 5"); });

  master.start_bus();

  master.run(std::chrono::microseconds(1000));

  // auto TxPDO = master.getTxPDO(master.slaves[1]);
  auto &SlavePD = master.slaves_data_access[1];
  for(auto && da : master.slaves_data_access)
  {
    da.receive_mbx = true;
  }
  // SlavePD.receive_mbx = true;

  int cycles = 5;
  while (cycles--)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger(__logger_name), "cycle count: %d", master.get_cycle_counter());

    master.transfer_TxPDO();
    // printf("cycle count: %d\n", master.get_cycle_counter());
    hexdump(SlavePD.TxPDO.data(), SlavePD.TxPDO.size());

    if(SlavePD.receive_mbx_has_unread.load())
    {
      RCLCPP_INFO(rclcpp::get_logger(__logger_name), "mailbox has data");
      hexdump(SlavePD.RMbx.data(), SlavePD.RMbx.size());
      SlavePD.receive_mbx_has_unread.store(false);
    }
  }

  // //////////////////////////////////////////////////
  // // youbot trinamic controller mailbox structures
  // struct mailboxOutputBuffer
  // {
  //   uint8 moduleAddress; // 0 = Drive  1 = Gripper
  //   uint8 commandNumber;
  //   uint8 typeNumber;
  //   uint8 motorNumber; // always zero
  //   uint32 value;      // MSB first!

  //   mailboxOutputBuffer() : moduleAddress(0), commandNumber(0), typeNumber(0), motorNumber(0), value(0){};
  // } __attribute__((__packed__));

  // struct mailboxInputBuffer
  // {
  //   uint8 replyAddress;
  //   uint8 moduleAddress;
  //   uint8 status; //(e.g. 100 means “no error”)
  //   uint8 commandNumber;
  //   uint32 value; // MSB first!

  //   mailboxInputBuffer() : replyAddress(0), moduleAddress(0), status(0), commandNumber(0), value(0){};
  // } __attribute__((__packed__));
  // //////////////////////////////////////////////////

  // mailboxOutputBuffer smbx{};
  // smbx.commandNumber = 6; // GAP
  // smbx.typeNumber = 1; // position



  master.stop();
  RCLCPP_INFO(rclcpp::get_logger(__logger_name), "cycle counter: %d", master.get_cycle_counter());

  return EXIT_SUCCESS;
}
