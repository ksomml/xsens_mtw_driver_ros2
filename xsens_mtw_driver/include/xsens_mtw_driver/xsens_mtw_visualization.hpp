#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "imu_msgs/msg/imu_data_array.hpp"
#include "imu_msgs/msg/imu_data_single.hpp"


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
        bool m_oneTopicPerImu;
        bool m_useImuMapping;
        double m_imuSpacing;
        std::string m_mtwTopicName;
        std::string m_imuPrefix;
        std::vector<std::string> m_imuMapping;
        std::vector<std::string> m_imuIdsVec;

        rclcpp::Subscription<imu_msgs::msg::IMUDataArray>::SharedPtr m_imuMonoTopicSubscriber;
        std::vector<rclcpp::Subscription<imu_msgs::msg::IMUDataSingle>::SharedPtr> m_imuSubscriberArray;

        std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;

        std::set<std::string> m_subscribedTopics;
        rclcpp::TimerBase::SharedPtr m_topicCheckTimer;

        void discoverAndSubscribeToImuTopics();
        void imuDataDefaultMonoTopicCallback(const imu_msgs::msg::IMUDataArray::SharedPtr);
        void imuDataDefaultMultiTopicCallback(const imu_msgs::msg::IMUDataSingle::SharedPtr);
        void imuDataCustomMonoTopicCallback(const imu_msgs::msg::IMUDataArray::SharedPtr);
    };

} /* namespace xsens_mtw_visualization */