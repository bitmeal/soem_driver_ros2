#include <gtest/gtest.h>

#include <string>

#include <pluginlib/class_loader.hpp>
#include "soem_driver_slave_interface/soem_driver_slave.hpp"

TEST(TestLoadSOEMMockModule, load_plugin)
{
  pluginlib::ClassLoader<soem_driver_slave_interface::SOEMDriverSlave> loader{
      "soem_driver_slave_interface", "soem_driver_slave_interface::SOEMDriverSlave"};

  std::shared_ptr<soem_driver_slave_interface::SOEMDriverSlave> module;

  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));
  ASSERT_NO_THROW(module->init("mock", {}));
}
