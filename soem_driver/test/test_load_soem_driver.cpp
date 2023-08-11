#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "ros2_control_test_assets/descriptions.hpp"

// MATCHERS
using ::testing::AllOf;
// using ::testing::Gt;
// using ::testing::Lt;
// using ::testing::StartsWith;
using ::testing::HasSubstr;
using ::testing::ContainsRegex;
using ::testing::MatchesRegex;

// PREDICATES
using ::testing::ExitedWithCode;

TEST(TestLoadSOEMDriver, load_unconfigured_plugin)
{
  std::string hardware_system =
      R"(
  <ros2_control name="GenericSystem" type="system">
    <hardware>
      <plugin>soem_driver/SOEMDriver</plugin>
    </hardware>
  </ros2_control>
)";

  auto urdf = ros2_control_test_assets::urdf_head +
              hardware_system +
              ros2_control_test_assets::urdf_tail;

  hardware_interface::ResourceManager rm;

  ASSERT_NO_THROW(rm.load_urdf(urdf));
  ASSERT_EQ(rm.system_components_size(), (size_t)1);
  ASSERT_EQ(
      rm.get_components_status().begin()->second.state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST(TestLoadSOEMDriver, load_configured_plugin)
{
  std::string hardware_system =
      R"(
  <ros2_control name="GenericSystem" type="system">
    <hardware>
      <plugin>soem_driver/SOEMDriver</plugin>
      <ec_interface>dummy0</ec_interface>
      <ec_cycle_us>1000</ec_cycle_us>

      <ec_timeout_pd_us>300</ec_timeout_pd_us>
      <ec_timeout_mbxrx_us>200</ec_timeout_mbxrx_us>
      <ec_timeout_mbxtx_us>100</ec_timeout_mbxtx_us>
    </hardware>
  </ros2_control>
)";

  auto urdf = ros2_control_test_assets::urdf_head +
              hardware_system +
              ros2_control_test_assets::urdf_tail;

  hardware_interface::ResourceManager rm;

  ASSERT_NO_THROW(rm.load_urdf(urdf));
  ASSERT_EQ(rm.system_components_size(), (size_t)1);
  ASSERT_EQ(
      rm.get_components_status().begin()->second.state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(TestLoadSOEMDriver, load_configured_plugin_config)
{
  EXPECT_EXIT(
      {
        std::string hardware_system =
            R"(
  <ros2_control name="GenericSystem" type="system">
    <hardware>
      <plugin>soem_driver/SOEMDriver</plugin>
      <ec_interface>dummy0</ec_interface>
      <ec_cycle_us>1000</ec_cycle_us>

      <ec_timeout_pd_us>300</ec_timeout_pd_us>
      <ec_timeout_mbxrx_us>200</ec_timeout_mbxrx_us>
      <ec_timeout_mbxtx_us>100</ec_timeout_mbxtx_us>
    </hardware>
  </ros2_control>
)";

        auto urdf = ros2_control_test_assets::urdf_head +
                    hardware_system +
                    ros2_control_test_assets::urdf_tail;

        hardware_interface::ResourceManager rm;
        ASSERT_NO_THROW(rm.load_urdf(urdf));

        exit(0);
      },
      ExitedWithCode(0),
      AllOf(
          ContainsRegex("interface.*dummy0"),
          ContainsRegex("EtherCAT.*1000µs"),
          ContainsRegex("process.*data.*300"),
          ContainsRegex("mailbox.*receive.*200"),
          ContainsRegex("mailbox.*send.*100")));
}

TEST(TestLoadSOEMDriver, load_generic_system_configured_no_joints)
{
  std::string hardware_system =
      R"(
  <ros2_control name="GenericSystem" type="system">
    <hardware>
      <plugin>soem_driver/SOEMDriver</plugin>
      <ec_interface>dummy0</ec_interface>
      <ec_cycle_us>1000</ec_cycle_us>

      <ec_slave name="module_0">
          <alias>0</alias>
          <position>0</position>
          <plugin>soem_slave_modules/soem_mock_module</plugin>
      </ec_slave>
    </hardware>
  </ros2_control>
)";

  auto urdf = ros2_control_test_assets::urdf_head +
              hardware_system +
              ros2_control_test_assets::urdf_tail;

  hardware_interface::ResourceManager rm;

  ASSERT_NO_THROW(rm.load_urdf(urdf));
  ASSERT_EQ(rm.system_components_size(), (size_t)1);
  ASSERT_EQ(
      rm.get_components_status().begin()->second.state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(TestLoadSOEMDriver, load_generic_system_configured)
{
  std::string hardware_system =
      R"(
  <ros2_control name="GenericSystem" type="system">
    <hardware>
      <plugin>soem_driver/SOEMDriver</plugin>
      <ec_interface>dummy0</ec_interface>
      <ec_cycle_us>1000</ec_cycle_us>

      <ec_slave name="module_0">
          <alias>0</alias>
          <position>0</position>
          <plugin>soem_slave_modules/soem_mock_module</plugin>
      </ec_slave>
      <ec_slave name="module_1">
          <alias>0</alias>
          <position>1</position>
          <plugin>soem_slave_modules/soem_mock_module</plugin>
      </ec_slave>
    </hardware>

    <joint name="joint_0">
        <param name="ec_claims">module_0/joint_unit</param>
    </joint>
    <joint name="joint_1">
        <param name="ec_claims">
          [
            module_0/sensor_unit,
            module_1/actuator_unit
          ]
        </param>
    </joint>
    <joint name="joint_2">
        <param name="ec_claims">
            - module_1/sensor_unit/state/position
            - module_1/sensor_unit/state/velocity
            - module_1/sensor_unit/state/effort
            - module_1/joint_unit
        </param>
    </joint>
  </ros2_control>
)";

  auto urdf = ros2_control_test_assets::urdf_head +
              hardware_system +
              ros2_control_test_assets::urdf_tail;

  hardware_interface::ResourceManager rm;

  ASSERT_NO_THROW(rm.load_urdf(urdf));
  ASSERT_EQ(rm.system_components_size(), (size_t)1);
  ASSERT_EQ(
      rm.get_components_status().begin()->second.state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}
