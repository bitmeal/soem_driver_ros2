#include <gtest/gtest.h>

#include <string>

#include <rclcpp/time.hpp>
#include <pluginlib/class_loader.hpp>

#include "soem_slave_interface/soem_slave.hpp"

TEST(TestSOEMMockModuleInterface, call_init)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  ASSERT_NO_THROW(module->init({}));
}

TEST(TestSOEMMockModuleInterface, call_export_state_interfaces)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  ASSERT_NO_THROW(module->export_state_interfaces());
}

TEST(TestSOEMMockModuleInterface, call_export_command_interfaces)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  ASSERT_NO_THROW(module->export_command_interfaces());
}

TEST(TestSOEMMockModuleInterface, call_configure)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  ASSERT_NO_THROW(module->configure(0x00, 0x00, 0x00, {}));
}

TEST(TestSOEMMockModuleInterface, call_setup_SDO_hook)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  ASSERT_NO_THROW(module->setup_SDO_hook(soem_slave_interface::SOEMSlave::SDOwrite_t{}));
}

TEST(TestSOEMMockModuleInterface, call_read)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  ASSERT_NO_THROW(module->read(rclcpp::Time(0), rclcpp::Duration(0, 0)));
}

TEST(TestSOEMMockModuleInterface, call_write)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  ASSERT_NO_THROW(module->write(rclcpp::Time(0), rclcpp::Duration(0, 0)));
}

TEST(TestSOEMMockModuleData, export_state_interfaces)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  std::vector<hardware_interface::StateInterface> state_interfaces;
  ASSERT_NO_THROW(state_interfaces = module->export_state_interfaces());

  std::vector<std::string> expected_interfaces{
      "joint_unit/position",
      "joint_unit/velocity",
      "joint_unit/effort",

      "sensor_unit/position",
      "sensor_unit/velocity",
      "sensor_unit/effort",
  };

  std::for_each(expected_interfaces.begin(), expected_interfaces.end(), [&](auto &&expected)
                {
                  std::cout << "testing for: " << expected << std::endl;
                  ASSERT_TRUE(
                      std::any_of(state_interfaces.begin(), state_interfaces.end(), [&](auto &&exported)
                                  { return exported.get_name() == expected; })); });
}

TEST(TestSOEMMockModuleData, read_state_interfaces)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  std::vector<hardware_interface::StateInterface> state_interfaces;
  ASSERT_NO_THROW(state_interfaces = module->export_state_interfaces());

  std::for_each(state_interfaces.begin(), state_interfaces.end(), [&](auto &&interface)
                { ASSERT_NO_THROW(interface.get_value()); });
}

TEST(TestSOEMMockModuleData, export_command_interfaces)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  ASSERT_NO_THROW(command_interfaces = module->export_command_interfaces());

  std::vector<std::string> expected_interfaces{
      "joint_unit/position",
      "joint_unit/velocity",
      "joint_unit/effort",

      "actuator_unit/position",
      "actuator_unit/velocity",
      "actuator_unit/effort",
  };

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
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  ASSERT_NO_THROW(command_interfaces = module->export_command_interfaces());

  std::for_each(command_interfaces.begin(), command_interfaces.end(), [&](auto &&interface)
                { ASSERT_NO_THROW(interface.get_value()); });
}

TEST(TestSOEMMockModuleData, write_command_interfaces)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;
  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  ASSERT_NO_THROW(command_interfaces = module->export_command_interfaces());

  std::for_each(command_interfaces.begin(), command_interfaces.end(), [&](auto &&interface)
                { ASSERT_NO_THROW(interface.set_value(0)); });
}

