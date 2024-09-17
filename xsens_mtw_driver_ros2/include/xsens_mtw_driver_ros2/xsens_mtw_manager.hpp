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
#include "imu_msgs/msg/quaternion.hpp"

// Driver
#include "mastercallback.hpp"
#include "mtwcallback.hpp"

// Xsens
#include "../include/xsens/xsensdeviceapi.h"  // The Xsens device API header
#include "../include/xsens/xstypes.h"
#include "../include/xsens/xsmutex.h"
#include "../include/xsens/conio.h"  // For non ANSI _kbhit() and _getch()

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

namespace stvs = std_srvs::srv;

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

private:
    HardwareStatus status_;
    std::string file_name_;
    std::ofstream file_;
    std::vector<int> data_tracker_;
    std::vector<bool> data_skip_announced_;
    int max_data_skip_;
    bool waitForConnections_;
    bool key_interrupt_;
    bool isHeaderWritten_;

    // ROS2 Parameters
    std::string topic_name_;
    double ros2_rate_;
    int imu_rate_;
    int radio_channel_;
    bool useMagnetometer_;

    // ROS2 Callbacks
    rclcpp::TimerBase::SharedPtr connectTimer_;
    rclcpp::TimerBase::SharedPtr publishTimer_;

    // ROS2 Publisher
    rclcpp::Publisher<imu_msgs::msg::IMUDataArray>::SharedPtr imu_pub_;
    int64_t time_elapsed_;

    // ROS2 Services
    rclcpp::Service<stvs::Trigger>::SharedPtr status_service_;
    rclcpp::Service<stvs::Trigger>::SharedPtr get_ready_service_;
    rclcpp::Service<stvs::Trigger>::SharedPtr start_service_;
    rclcpp::Service<stvs::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<stvs::Trigger>::SharedPtr imu_reset_service_;
    rclcpp::Service<stvs::Trigger>::SharedPtr restart_service_;
    bool restart_requested_;

    // Xsens
    size_t connectedMTWCount_;
    WirelessMasterCallback wirelessMasterCallback_;
    std::vector<MtwCallback *> mtwCallbacks_;
    XsControl * control_;
    XsPortInfoArray detectedDevices_;
    XsPortInfoArray::const_iterator wirelessMasterPort_;
    XsDevicePtr wirelessMasterDevice_;
    XsDeviceIdArray allDeviceIds_;
    XsDeviceIdArray mtwDeviceIds_;
    XsDevicePtrArray mtwDevices_;

    // VQF
    std::vector<std::pair<int, VQF>> vqfContainer_;
    std::vector<imu_msgs::msg::IMUData> imu_data_msg_;


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
        const std::shared_ptr<stvs::Trigger::Request> request = nullptr);
    void stopRecording();
    void resetIMUs();
    void statusCallback(const std::shared_ptr<stvs::Trigger::Request>,
        std::shared_ptr<stvs::Trigger::Response>);
    void getReadyCallback(const std::shared_ptr<stvs::Trigger::Request>,
        std::shared_ptr<stvs::Trigger::Response>);
    void startRecordingCallback(const std::shared_ptr<stvs::Trigger::Request>,
        std::shared_ptr<stvs::Trigger::Response>);
    void stopRecordingCallback(const std::shared_ptr<stvs::Trigger::Request>,
        std::shared_ptr<stvs::Trigger::Response>);
    void imuResetCallback(const std::shared_ptr<stvs::Trigger::Request>,
        std::shared_ptr<stvs::Trigger::Response>);
    void restartDriverCallback(const std::shared_ptr<stvs::Trigger::Request>,
        std::shared_ptr<stvs::Trigger::Response>);
    void handleError(std::string error);
    std::string getHardwareStatusString();
};


/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream & operator << (std::ostream & out, XsPortInfo const & p);

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream & operator << (std::ostream & out, XsDevice const & d);

}  /* namespace xsens_mtw_manager */
