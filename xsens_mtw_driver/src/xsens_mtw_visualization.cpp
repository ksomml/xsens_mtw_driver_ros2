#include "../include/xsens_mtw_driver/xsens_mtw_visualization.hpp"

namespace xsens_mtw_visualization
{
    XsensVisualization::XsensVisualization(const std::string& name)
        : Node(name)
    {
        // Initialize variables
        bool received_parameters = false;

        // Parameters
        this->declare_parameter("imu_prefix", "None");
        this->get_parameter("imu_prefix", imu_prefix_);

        RCLCPP_INFO(this->get_logger(), "IMU prefix: %s", imu_prefix_.c_str());

        this->declare_parameter("imu_mapping", std::vector<std::string>{});
        this->get_parameter("imu_mapping", imu_mapping_);

        if (imu_prefix_ != "None" && !imu_mapping_.empty()) received_parameters = true;
        else received_parameters = false;

        // Initialize subscriber
        if (received_parameters)
        {
            RCLCPP_INFO(this->get_logger(), "IMU mapping received. Using custom 8 IMU lower body visualization. (Base IMU: %s)", (imu_prefix_ + imu_mapping_[0]).c_str());
            xsens_imu_data_sub_ = this->create_subscription<imu_msgs::msg::IMUDataArray>("xsens_imu_data", 10, std::bind(&XsensVisualization::imu_data_callback_custom, this, _1));

            // Generate full IMU IDs
            std::string imu_id_str, imu_ids_str;
            for (size_t i = 0; i < imu_mapping_.size(); i += 2)
            {
                imu_id_str = imu_prefix_ + imu_mapping_[i];
                imu_ids_vec_.push_back(imu_id_str);
                imu_ids_str += imu_id_str + ", ";
            }

            RCLCPP_INFO(this->get_logger(), "IMU IDs: %s", imu_ids_str.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No IMU mapping received. Publishing IMUs next to each other.");
            xsens_imu_data_sub_ = this->create_subscription<imu_msgs::msg::IMUDataArray>("xsens_imu_data", 10, std::bind(&XsensVisualization::imu_data_callback_default, this, _1));
        }

        // TF
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


        RCLCPP_INFO(this->get_logger(), "Broadcasting TF for RVIZ.");
    }

    XsensVisualization::~XsensVisualization() = default;


    void XsensVisualization::imu_data_callback_custom(const imu_msgs::msg::IMUDataArray::SharedPtr msg)
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        for (size_t i = 0; i < msg->imu_data.size(); i++)
        {
            geometry_msgs::msg::TransformStamped t;
            std::string id = msg->imu_data[i].id;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";  // 0 is base IMU
            t.child_frame_id = id;

            // Add your new IMUs here - Make sure to modify config/imu_mapping.yaml accordingly
            if (id == imu_ids_vec_[0]) { // pelvis_imu
                t.transform.translation.z = 1.5;
            }
            else if (id == imu_ids_vec_[1]) { // torso_imu
                t.transform.translation.z = 2.0;
            }
            else if (id == imu_ids_vec_[2]) { // femur_l_imu
                t.transform.translation.z = 1.0;
                t.transform.translation.y = 0.5;
            }
            else if (id == imu_ids_vec_[3]) { // femur_r_imu
                t.transform.translation.z = 1.0;
                t.transform.translation.y = -0.5;
            }
            else if (id == imu_ids_vec_[4]) { // tibia_l_imu
                t.transform.translation.z = 0.5;
                t.transform.translation.y = 0.5;
            }
            else if (id == imu_ids_vec_[5]) { // tibia_r_imu
                t.transform.translation.z = 0.5;
                t.transform.translation.y = -0.5;
            }
            else if (id == imu_ids_vec_[6]) { // calcn_l_imu
                t.transform.translation.y = 0.5;
            }
            else if (id == imu_ids_vec_[7]) { // calcn_r_imu
                t.transform.translation.y = -0.5;
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Unknown IMU ID: %s", id.c_str());
            }

            t.transform.rotation.w = msg->imu_data[i].orientation.w;
            t.transform.rotation.x = msg->imu_data[i].orientation.x;
            t.transform.rotation.y = msg->imu_data[i].orientation.y;
            t.transform.rotation.z = msg->imu_data[i].orientation.z;

            transforms.push_back(t);
        }

        tf_broadcaster_->sendTransform(transforms);
    }

    void XsensVisualization::imu_data_callback_default(const imu_msgs::msg::IMUDataArray::SharedPtr msg)
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        for (size_t i = 0; i < msg->imu_data.size(); i++)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = msg->imu_data[i].id;

            t.transform.translation.y = i / 2.0 + 0.5;    // just to add some space between the IMUs

            t.transform.rotation.w = msg->imu_data[i].orientation.w;
            t.transform.rotation.x = msg->imu_data[i].orientation.x;
            t.transform.rotation.y = msg->imu_data[i].orientation.y;
            t.transform.rotation.z = msg->imu_data[i].orientation.z;

            transforms.push_back(t);
        }

        tf_broadcaster_->sendTransform(transforms);
    }

} /* namespace xsens_mtw_visualization */