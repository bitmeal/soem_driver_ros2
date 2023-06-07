#include <gtest/gtest.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "ros2_control_test_assets/descriptions.hpp"

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

TEST(TestLoadSOEMDriver, load_generic_system_configured_no_joints)
{
  std::string hardware_system =
      R"(
  <ros2_control name="GenericSystem" type="system">
    <hardware>
      <plugin>soem_driver/SOEMDriver</plugin>
      <ec_interface>eth1</ec_interface>
      <ec_slave name="controller_0">
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
      <ec_interface>eth1</ec_interface>
      <ec_slave name="controller_0">
          <alias>0</alias>
          <position>0</position>
          <plugin>soem_slave_modules/soem_mock_module</plugin>
      </ec_slave>
    </hardware>

    <joint name="joint_0">
        <param name="ec_claims">controller_0/drive_0</param>
    </joint>
    <joint name="joint_1">
        <param name="ec_claims">
          [
            controller_0/drive_1,
            controller_n/feedback_0
          ]
        </param>
    </joint>
    <joint name="joint_2">
        <param name="ec_claims">
            - controller_2/drive_0
            - controller_2/feedback_0
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
