#pragma once

// Standard
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// Custom ROS2 messages
#include "imu_msgs/msg/imu_data.hpp"
#include "imu_msgs/msg/imu_data_array.hpp"
#include "imu_msgs/msg/imu_data_single.hpp"
#include "imu_msgs/msg/quaternion.hpp"

// Custom ROS2
#include "xsens_srvs/srv/trigger.hpp"
#include "xsens_srvs/srv/start_recording.hpp"

// Driver
#include "mastercallback.hpp"
#include "mtwcallback.hpp"

// XSens
#include "../xsens/xsensdeviceapi.h"  // The Xsens device API header
#include "../xsens/xstypes.h"
#include "../xsens/xsmutex.h"
#include "../xsens/conio.h"  // For non ANSI _kbhit() and _getch()

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

namespace xsrvs = xsens_srvs::srv;

enum HardwareStatus
{
    ERROR = -2,
    NO_CONNECTION = -1,
    OK = 0,
    READY = 1,
    RECORDING = 2
};


class XsensManager : public rclcpp::Node
{
public:
    explicit XsensManager(const std::string & name);
    virtual ~XsensManager();
    void cleanupAndShutdown();
    HardwareStatus getStatus() { return m_status; }

private:
    HardwareStatus m_status;
    std::string m_fileName;
    std::ofstream m_file;
    std::vector<int> m_dataTracker;
    int m_maxDataSkip;
    int64_t m_timestamp;
    bool m_waitForConnections;
    bool m_keyInterrupt;
    bool m_isHeaderWritten;

    // ROS2 Parameters
    std::string m_mtwTopicName;
    bool m_oneTopicPerImu;
    int m_ros2Rate;
    int m_imuRate;
    int m_radioChannel;
    bool m_imuResetOnRecord;
    bool m_useMagnetometer;

    // ROS2 Callbacks
    rclcpp::TimerBase::SharedPtr m_connectTimer;
    rclcpp::TimerBase::SharedPtr m_publishTimer;

    // ROS2 Publisher
    rclcpp::Publisher<imu_msgs::msg::IMUDataArray>::SharedPtr m_imuPublisher;
    std::vector<rclcpp::Publisher<imu_msgs::msg::IMUDataSingle>::SharedPtr> m_imuPublishers;

    // ROS2 Services
    rclcpp::Service<xsrvs::Trigger>::SharedPtr m_statusService;
    rclcpp::Service<xsrvs::Trigger>::SharedPtr m_getReadyService;
    rclcpp::Service<xsrvs::StartRecording>::SharedPtr m_startService;
    rclcpp::Service<xsrvs::Trigger>::SharedPtr m_stopService;
    rclcpp::Service<xsrvs::Trigger>::SharedPtr m_imuResetService;
    rclcpp::Service<xsrvs::Trigger>::SharedPtr m_restartService;
    bool m_restartRequested;

    // Xsens
    size_t m_connectedMTWCount;
    WirelessMasterCallback m_wirelessMasterCallback;
    std::vector<MtwCallback *> m_mtwCallbacks;
    XsControl * m_control;
    XsPortInfoArray m_detectedDevices;
    XsPortInfoArray::const_iterator m_wirelessMasterPort;
    XsDevicePtr m_wirelessMasterDevice;
    XsDeviceIdArray m_allDeviceIds;
    XsDeviceIdArray m_mtwDeviceIds;
    XsDevicePtrArray m_mtwDevices;

    // VQF
    std::vector<VQF> m_vqfFilters;
    std::vector<imu_msgs::msg::IMUData> m_imuDataMsg;

    int getMaxUpdateRate(int);
    int findClosestUpdateRate(const XsIntArray &, const int);
    void initialMasterSetup();
    void connectMTWsCallback();
    void completeInitialization();
    void publishDataCallback();
    void checkRateSupport();
    void mtwSetup();
    void vqfSetup();
    void rosMessagesSetup();
    void generateFileName();
    void writeFileHeader();
    void writeDataToFile();
    void closeFile();
    void startRecording(
        const std::shared_ptr<xsrvs::StartRecording::Request> t_request = nullptr);
    void stopRecording();
    void resetIMUs();
    void statusCallback(const std::shared_ptr<xsrvs::Trigger::Request>,
        std::shared_ptr<xsrvs::Trigger::Response>);
    void getReadyCallback(const std::shared_ptr<xsrvs::Trigger::Request>,
        std::shared_ptr<xsrvs::Trigger::Response>);
    void startRecordingCallback(const std::shared_ptr<xsrvs::StartRecording::Request>,
        std::shared_ptr<xsrvs::StartRecording::Response>);
    void stopRecordingCallback(const std::shared_ptr<xsrvs::Trigger::Request>,
        std::shared_ptr<xsrvs::Trigger::Response>);
    void imuResetCallback(const std::shared_ptr<xsrvs::Trigger::Request>,
        std::shared_ptr<xsrvs::Trigger::Response>);
    void restartDriverCallback(const std::shared_ptr<xsrvs::Trigger::Request>,
        std::shared_ptr<xsrvs::Trigger::Response>);
    void logAndRaiseSigInt(std::string);
};


/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream & operator << (std::ostream & out, XsPortInfo const & p);

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream & operator << (std::ostream & out, XsDevice const & d);

}  // namespace xsens_mtw_manager
