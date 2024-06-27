#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "imu_msgs/msg/imu_data_array.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace xsens_mtw_visualization
{
    class XsensVisualization : public rclcpp::Node
    {
    public:
        XsensVisualization(const std::string& name);
        virtual ~XsensVisualization();

    private:
        std::string imu_prefix_;
        std::vector<std::string> imu_mapping_;
        std::vector<std::string> imu_ids_vec_;

        rclcpp::Subscription<imu_msgs::msg::IMUDataArray>::SharedPtr xsens_imu_data_sub_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        void imu_data_callback_default(const imu_msgs::msg::IMUDataArray::SharedPtr);
        void imu_data_callback_custom(const imu_msgs::msg::IMUDataArray::SharedPtr);
    };

} /* namespace xsens_mtw_visualization */