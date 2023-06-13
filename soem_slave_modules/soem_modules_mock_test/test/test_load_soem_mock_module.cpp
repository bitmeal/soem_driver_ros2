#include <gtest/gtest.h>

#include <string>

#include <pluginlib/class_loader.hpp>
#include "soem_slave_interface/soem_slave.hpp"

TEST(TestLoadSOEMMockModule, load_plugin)
{
  pluginlib::ClassLoader<soem_slave_interface::SOEMSlave> loader{
      "soem_slave_interface", "soem_slave_interface::SOEMSlave"};

  std::shared_ptr<soem_slave_interface::SOEMSlave> module;

  ASSERT_NO_THROW(module = loader.createSharedInstance("soem_slave_modules/soem_mock_module"));
  ASSERT_NO_THROW(module->init({}));
}
