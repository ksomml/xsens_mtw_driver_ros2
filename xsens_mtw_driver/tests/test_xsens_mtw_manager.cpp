#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include "../include/xsens_mtw_driver/xsens_mtw_manager.hpp"


class XsensMtwManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node = std::make_shared<xsens_mtw_manager::XsensManager>("test_node");

        // Use a lambda to declare or set parameters conditionally.
        auto assign_param = [this](const std::string &name, const rclcpp::ParameterValue &value) {
            if (!node->has_parameter(name)) {
                node->declare_parameter(name, value);
            } else {
                node->set_parameter(rclcpp::Parameter(name, value));
            }
        };

        assign_param("topic_name", rclcpp::ParameterValue("xsens_imu_data"));
        assign_param("ros2_rate", rclcpp::ParameterValue(100));
        assign_param("imu_rate", rclcpp::ParameterValue(100));
        assign_param("radio_channel", rclcpp::ParameterValue(25));
        assign_param("imu_reset_on_record", rclcpp::ParameterValue(true));
        assign_param("use_magnetometer", rclcpp::ParameterValue(false));
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    std::shared_ptr<xsens_mtw_manager::XsensManager> node;
};


TEST_F(XsensMtwManagerTest, TestInitialization)
{
    ASSERT_EQ(node->getStatus(), xsens_mtw_manager::HardwareStatus::OK);
    ASSERT_EQ(node->get_parameter("topic_name").as_string(), "xsens_imu_data");
    ASSERT_EQ(node->get_parameter("ros2_rate").as_int(), 100);
    ASSERT_EQ(node->get_parameter("imu_rate").as_int(), 100);
    ASSERT_EQ(node->get_parameter("radio_channel").as_int(), 25);
    ASSERT_EQ(node->get_parameter("imu_reset_on_record").as_bool(), true);
    ASSERT_EQ(node->get_parameter("use_magnetometer").as_bool(), false);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
