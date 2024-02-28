#include <gtest/gtest.h>

#include <string>

#include <rclcpp/time.hpp>

#include <pluginlib/class_loader.hpp>
#include "soem_driver_slave_interface/soem_driver_slave.hpp"

TEST(TestSOEMModuleTMCM1610, load_plugin)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;

  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));
  ASSERT_NO_THROW(module->init("trinamic_tmcm1610_test", {}));
}

TEST(TestSOEMModuleTMCM1610, call_init)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  ASSERT_NO_THROW(module->init("trinamic_tmcm1610_test", {}));
}

TEST(TestSOEMModuleTMCM1610, call_export_state_interfaces)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  ASSERT_NO_THROW(module->export_state_interfaces());
}

TEST(TestSOEMModuleTMCM1610, call_export_command_interfaces)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  ASSERT_NO_THROW(module->export_command_interfaces());
}

TEST(TestSOEMModuleTMCM1610, call_configure_wrong_id)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  ASSERT_EQ(module->configure(0x00, 0x00, 0x00, {}), false);
}

TEST(TestSOEMModuleTMCM1610, call_configure_incomplete)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  ASSERT_EQ(module->configure(0x0286, 0x0070, 0x00, {}), false);
}

TEST(TestSOEMModuleTMCM1610, call_configure)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  ASSERT_EQ(
    module->configure(
      0x0286, 0x0070, 0x00,
      {
        {"gear_ratio", "0"},
        {"torque_constant", "0"}
      }
    ), true);
}

// TODO(bitmeal): implement error handling for type conversion; this test well detect changes in implementation
TEST(TestSOEMModuleTMCM1610, call_configure_wrong_type)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  ASSERT_ANY_THROW(
    module->configure(
      0x0286, 0x0070, 0x00,
      {
        {"gear_ratio", "not a number"},
        {"torque_constant", "not a number"}
      }
    ));
}

TEST(TestSOEMModuleTMCM1610, call_setup_SDO_hook)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  ASSERT_NO_THROW(module->setup_SDO_hook(soem_driver::SDOwrite_t{}));
}

TEST(TestSOEMModuleTMCM1610, call_read)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  // interface data structures are only allocated after call to export
  ASSERT_NO_THROW(module->export_state_interfaces());
  ASSERT_NO_THROW(module->read(rclcpp::Time(0), rclcpp::Duration(0, 0)));
}

TEST(TestSOEMModuleTMCM1610, call_write)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  // interface data structures are only allocated after call to export
  ASSERT_NO_THROW(module->export_command_interfaces());
  ASSERT_NO_THROW(module->write(rclcpp::Time(0), rclcpp::Duration(0, 0)));
}

TEST(TestSOEMMockModuleData, export_state_interfaces)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  std::vector<hardware_interface::StateInterface> state_interfaces;
  ASSERT_NO_THROW(state_interfaces = module->export_state_interfaces());

  std::vector<std::string> expected_interfaces{
      "joint/position",
      "joint/velocity",
      "joint/effort",

      "gripper/position"};

  std::for_each(expected_interfaces.begin(), expected_interfaces.end(), [&](auto &&expected)
                {
                  std::cout << "testing for: " << expected << std::endl;
                  ASSERT_TRUE(
                      std::any_of(state_interfaces.begin(), state_interfaces.end(), [&](auto &&exported)
                                  { return exported.get_name() == expected; })); });
}

TEST(TestSOEMMockModuleData, read_state_interfaces)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  std::vector<hardware_interface::StateInterface> state_interfaces;
  ASSERT_NO_THROW(state_interfaces = module->export_state_interfaces());

  std::for_each(state_interfaces.begin(), state_interfaces.end(), [&](auto &&interface)
                { ASSERT_NO_THROW(interface.get_value()); });
}

TEST(TestSOEMMockModuleData, export_command_interfaces)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  ASSERT_NO_THROW(command_interfaces = module->export_command_interfaces());

  std::vector<std::string> expected_interfaces{
      "joint/position",
      "joint/velocity",
      "joint/effort",

      "gripper/position"};

  std::for_each(expected_interfaces.begin(), expected_interfaces.end(), [&](auto &&expected)
                { 
                  std::cout << "testing for: " << expected << std::endl;
                  ASSERT_TRUE(
                      std::any_of(command_interfaces.begin(), command_interfaces.end(), [&](auto &&exported)
                                  { 
                                    return exported.get_name() == expected; })); });
}

TEST(TestSOEMMockModuleData, read_command_interfaces)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  ASSERT_NO_THROW(command_interfaces = module->export_command_interfaces());

  std::for_each(command_interfaces.begin(), command_interfaces.end(), [&](auto &&interface)
                { ASSERT_NO_THROW(interface.get_value()); });
}

TEST(TestSOEMMockModuleData, write_command_interfaces)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/trinamic_tmcm1610"));

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  ASSERT_NO_THROW(command_interfaces = module->export_command_interfaces());

  std::for_each(command_interfaces.begin(), command_interfaces.end(), [&](auto &&interface)
                { ASSERT_NO_THROW(interface.set_value(0)); });
}
