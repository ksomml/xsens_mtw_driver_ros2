#include "../include/xsens_mtw_driver/xsens_mtw_visualization.hpp"

namespace xsens_mtw_visualization
{
XsensVisualization::XsensVisualization(const std::string &name)
    : Node(name)
{
    // --------------------------------------------------------------------
    // ROS2 PARAMETERS
    this->declare_parameter("one_topic_per_imu", true);
    m_oneTopicPerImu = this->get_parameter("one_topic_per_imu").as_bool();

    this->declare_parameter("mtw_topic_name", "xsens_imu_data");
    m_mtwTopicName = this->get_parameter("mtw_topic_name").as_string();

    this->declare_parameter("imu_prefix", "");
    m_imuPrefix = this->get_parameter("imu_prefix").as_string();

    this->declare_parameter("imu_spacing", 0.5);
    m_imuSpacing = this->get_parameter("imu_spacing").as_double();

    this->declare_parameter("use_imu_mapping", false);
    m_useImuMapping = this->get_parameter("use_imu_mapping").as_bool();

    this->declare_parameter("imu_mapping", std::vector<std::string>{});
    m_imuMapping = this->get_parameter("imu_mapping").as_string_array();

    RCLCPP_INFO(this->get_logger(), "ROS2 parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "- one_topic_per_imu: %s", m_oneTopicPerImu ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "- mtw_topic_name: %s", m_mtwTopicName.c_str());
    RCLCPP_INFO(this->get_logger(), "- imu_spacing: %.2f", m_imuSpacing);
    RCLCPP_INFO(this->get_logger(), "- use_imu_mapping: %s", m_useImuMapping ? "true" : "false");
    if (m_useImuMapping)
        RCLCPP_INFO(this->get_logger(), "- imu_prefix: %s", m_imuPrefix.c_str());

    // --------------------------------------------------------------------
    // ROS2 SUBSCRIPTIONS
    if (m_useImuMapping && !m_imuMapping.empty())
    {
        if (m_oneTopicPerImu)
        {
            RCLCPP_INFO(this->get_logger(), "Using one topic per IMU with custom IMU mapping.");
            throw std::runtime_error("One topic per IMU with custom IMU mapping is not implemented.");
        }
        else
        {
            // Create subscriber based on IMU mapping and topic configuration
            RCLCPP_INFO(this->get_logger(), "Using custom IMU mapping with mono topic.");
            RCLCPP_INFO(this->get_logger(), "IMU mapping received. Using custom IMU visualization. (Base IMU: %s)", (m_imuPrefix + m_imuMapping[0]).c_str());
            m_imuMonoTopicSubscriber = this->create_subscription<imu_msgs::msg::IMUDataArray>(m_mtwTopicName, 10, std::bind(&XsensVisualization::imuDataCustomMonoTopicCallback, this, _1));

            // Generate full IMU IDs
            std::string imuIdStr, imuIdsStr;
            for (size_t i = 0; i < m_imuMapping.size(); i += 2)
            {
                imuIdStr = m_imuPrefix + m_imuMapping[i];
                m_imuIdsVec.push_back(imuIdStr);
                imuIdsStr += imuIdStr + ", ";
            }

            RCLCPP_INFO(this->get_logger(), "IMU Mapping IDs: %s", imuIdsStr.c_str());
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Publishing IMUs next to each other.");

        if (m_oneTopicPerImu)
        {
            RCLCPP_INFO(this->get_logger(), "Using one topic per IMU with dynamic discovery.");
            m_subscribedTopics.clear();
            m_topicCheckTimer = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&XsensVisualization::discoverAndSubscribeToImuTopics, this));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Using mono topic for all IMUs.");
            m_imuMonoTopicSubscriber = this->create_subscription<imu_msgs::msg::IMUDataArray>(m_mtwTopicName, 10, std::bind(&XsensVisualization::imuDataDefaultMonoTopicCallback, this, _1));
        }
    }

    // --------------------------------------------------------------------
    // ROS2 TF BROADCASTER
    m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Broadcasting TF for RVIZ.");
}

XsensVisualization::~XsensVisualization() = default;

// Publish IMU data as TFs in RVIZ using mono IMU topic with given spacing
void XsensVisualization::imuDataDefaultMonoTopicCallback(const imu_msgs::msg::IMUDataArray::SharedPtr msg)
{
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    for (size_t i = 0; i < msg->imu_data.size(); i++)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = msg->imu_data[i].id;

        t.transform.translation.y = (i + 1) * m_imuSpacing;

        t.transform.rotation.w = msg->imu_data[i].orientation.w;
        t.transform.rotation.x = msg->imu_data[i].orientation.x;
        t.transform.rotation.y = msg->imu_data[i].orientation.y;
        t.transform.rotation.z = msg->imu_data[i].orientation.z;

        transforms.push_back(t);
    }

    m_tfBroadcaster->sendTransform(transforms);
}

// Publish IMU data as TFs in RVIZ using multiple IMU topics with given spacing
void XsensVisualization::imuDataDefaultMultiTopicCallback(const imu_msgs::msg::IMUDataSingle::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped t;

    std::string id = msg->imu_data.id;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = id;

    // Use a static map to assign unique indices to IMUs based on their ID
    static std::map<std::string, size_t> id_to_index;
    if (id_to_index.find(id) == id_to_index.end()) {
        id_to_index[id] = id_to_index.size();
    }
    size_t index = id_to_index[id];

    t.transform.translation.y = (index + 1) * m_imuSpacing;
    t.transform.rotation.w = msg->imu_data.orientation.w;
    t.transform.rotation.x = msg->imu_data.orientation.x;
    t.transform.rotation.y = msg->imu_data.orientation.y;
    t.transform.rotation.z = msg->imu_data.orientation.z;

    m_tfBroadcaster->sendTransform(t);
}

// Publish IMU data as TFs in RVIZ with custom mapping
void XsensVisualization::imuDataCustomMonoTopicCallback(const imu_msgs::msg::IMUDataArray::SharedPtr msg)
{
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    for (size_t i = 0; i < msg->imu_data.size(); i++)
    {
        geometry_msgs::msg::TransformStamped t;
        std::string id = msg->imu_data[i].id;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map"; // 0 is base IMU
        t.child_frame_id = id;

        // MODIFY HERE - Make sure to modify config/imu_mapping.yaml accordingly
        if (id == m_imuIdsVec[0])
        { // pelvis_imu
            t.transform.translation.z = 1.5;
        }
        else if (id == m_imuIdsVec[1])
        { // torso_imu
            t.transform.translation.z = 2.0;
        }
        else if (id == m_imuIdsVec[2])
        { // femur_l_imu
            t.transform.translation.z = 1.0;
            t.transform.translation.y = 0.5;
        }
        else if (id == m_imuIdsVec[3])
        { // femur_r_imu
            t.transform.translation.z = 1.0;
            t.transform.translation.y = -0.5;
        }
        else if (id == m_imuIdsVec[4])
        { // tibia_l_imu
            t.transform.translation.z = 0.5;
            t.transform.translation.y = 0.5;
        }
        else if (id == m_imuIdsVec[5])
        { // tibia_r_imu
            t.transform.translation.z = 0.5;
            t.transform.translation.y = -0.5;
        }
        else if (id == m_imuIdsVec[6])
        { // calcn_l_imu
            t.transform.translation.y = 0.5;
        }
        else if (id == m_imuIdsVec[7])
        { // calcn_r_imu
            t.transform.translation.y = -0.5;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Unknown IMU ID: %s", id.c_str());
        }

        t.transform.rotation.w = msg->imu_data[i].orientation.w;
        t.transform.rotation.x = msg->imu_data[i].orientation.x;
        t.transform.rotation.y = msg->imu_data[i].orientation.y;
        t.transform.rotation.z = msg->imu_data[i].orientation.z;

        transforms.push_back(t);
    }

    m_tfBroadcaster->sendTransform(transforms);
}

// Discover and subscribe to IMU topics dynamically
void XsensVisualization::discoverAndSubscribeToImuTopics()
{
    std::map<std::string, std::vector<std::string>> topic_info;

    try
    {
        topic_info = this->get_topic_names_and_types();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception during topic discovery: %s", e.what());
        return;
    }

    auto basePattern = "/" + m_mtwTopicName + "_";

    for (const auto &[topic_name, topic_types] : topic_info)
    {
        if (topic_name.find(basePattern) == 0 &&
            m_subscribedTopics.find(topic_name) == m_subscribedTopics.end() &&
            !topic_types.empty())
        {
            bool is_imu_msg = false;
            for (const auto &type : topic_types)
            {
                if (type.find("imu_msgs") != std::string::npos)
                {
                    is_imu_msg = true;
                    break;
                }
            }

            if (is_imu_msg)
            {
                auto subscriber = this->create_subscription<imu_msgs::msg::IMUDataSingle>(
                    topic_name, 10,
                    std::bind(&XsensVisualization::imuDataDefaultMultiTopicCallback, this, std::placeholders::_1));
                m_imuSubscriberArray.push_back(subscriber);
                m_subscribedTopics.insert(topic_name);

                RCLCPP_INFO(this->get_logger(), "Subscribed to IMU topic: %s", topic_name.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Found potential IMU topic but message type doesn't match: %s", topic_name.c_str());
            }
        }
    }
}
} /* namespace xsens_mtw_visualization */