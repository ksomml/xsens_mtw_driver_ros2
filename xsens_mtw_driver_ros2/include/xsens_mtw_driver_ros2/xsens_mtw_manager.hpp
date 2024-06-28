#pragma once

// Standard
#include <fstream>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// Custom ROS 2 messages
#include "imu_msgs/msg/imu_data.hpp"
#include "imu_msgs/msg/imu_data_array.hpp"
#include "imu_msgs/msg/quaternion.hpp"

// Driver
#include "mastercallback.hpp"
#include "mtwcallback.hpp"
#include "findClosestUpdateRate.hpp"

// XSens
#include "../include/xsens/xsensdeviceapi.h" 	// The Xsens device API header
#include "../include/xsens/xstypes.h"
#include "../include/xsens/xsmutex.h"
#include "../include/xsens/conio.h"				// For non ANSI _kbhit() and _getch()

// VQF
#include "vqf.hpp"


/*

|   MTw | desiredUpdateRate (max) |
|-------|-------------------------|
|   1-5 |           120 Hz        |
|   6-9 |           100 Hz        |
|    10 |            80 Hz        |
| 11-20 |            60 Hz        |
| 21-32 |            40 Hz        |


*/


using std::placeholders::_1;
using std::placeholders::_2;

namespace xsens_mtw_manager
{
    enum HardwareStatus {
        ERROR = -2,
        NO_CONNECTION = -1,
        OK = 0,
        READY = 1,
        RECORDING = 2
    };

    class XSensManager : public rclcpp::Node
    {
    public:
        XSensManager(const std::string& name);
        virtual ~XSensManager();
        void cleanupAndShutdown();

    private:
        HardwareStatus status_;
        std::string file_name_;
        std::ofstream file_;
        bool waitForConnections_;
        bool interruption_;
        bool isHeaderWritten_;

        // ROS2 Parameters
        std::string topic_name_;
        double ros2_rate_;
        int imu_rate_;
        int radio_channel_;
        bool resetTimerOnRecord_;

        // ROS2 Callbacks
        rclcpp::TimerBase::SharedPtr connectTimer_;
        rclcpp::TimerBase::SharedPtr publishTimer_;

        // ROS2 Publisher
        rclcpp::Publisher<imu_msgs::msg::IMUDataArray>::SharedPtr imu_pub_;
        rclcpp::Time time_start_;
        double time_elapsed_;

        // ROS2 Services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_ready_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

        // XSens
        size_t connectedMTWCount_;
        WirelessMasterCallback wirelessMasterCallback_;
        std::vector<MtwCallback*> mtwCallbacks_;
        XsControl* control_;
        XsPortInfoArray detectedDevices_;
        XsPortInfoArray::const_iterator wirelessMasterPort_;
        XsDevicePtr wirelessMasterDevice_;
        XsDeviceIdArray allDeviceIds_;
        XsDeviceIdArray mtwDeviceIds_;
        XsDevicePtrArray mtwDevices_;

        // VQF
        std::vector<std::pair<int, VQF>> vqfContainer_;
        std::vector<imu_msgs::msg::IMUData> imu_data_msg_;

        // Debugging
        std::chrono::system_clock::time_point start_time_;
        std::chrono::system_clock::time_point end_time_;
        std::chrono::duration<double, std::milli> elapsed_time_;
        int counter_ = 0;


        void connectMTWsCallback();
        void publishDataCallback();
        void mtwSetup();
        void vqfSetup();
        void rosMessagesSetup();
        void generateFileName();
        void writeFileHeader();
        void writeDataToFile();
        void startRecording();
        void stopRecording();
        void statusCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response>);
        void getReadyCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response>);
        void startRecordingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response>);
        void stopRecordingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response>);
        void handleError(std::string error);
    };

    std::string hardwareStatusToString(HardwareStatus status);

    /*! \brief Stream insertion operator overload for XsPortInfo */
    std::ostream& operator << (std::ostream& out, XsPortInfo const& p);

    /*! \brief Stream insertion operator overload for XsDevice */
    std::ostream& operator << (std::ostream& out, XsDevice const& d);

} /* namespace xsens_mtw_manager */