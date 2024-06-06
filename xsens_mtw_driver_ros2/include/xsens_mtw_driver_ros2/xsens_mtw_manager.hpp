#pragma once

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

// ROS2
#include "rclcpp/rclcpp.hpp"

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


namespace xsens_mtw_manager
{
    class XsensManager : public rclcpp::Node
    {
    public:
        XsensManager(const std::string& name);
        virtual ~XsensManager();
        void cleanupAndShutdown();

    private:
        double ros2_rate;
        int imu_rate;
        int radio_channel;
        bool isRecording;
        std::string topic_name;

        // ROS2
        rclcpp::Publisher<imu_msgs::msg::IMUDataArray>::SharedPtr imu_pub;
        rclcpp::Time time_start;
        rclcpp::TimerBase::SharedPtr timer_;

        // XSens
        size_t connectedMTWCount;
        WirelessMasterCallback wirelessMasterCallback;
        std::vector<MtwCallback*> mtwCallbacks;
        XsControl* control;
        XsPortInfoArray detectedDevices;
        XsPortInfoArray::const_iterator wirelessMasterPort;
        XsDevicePtr wirelessMasterDevice;
        XsDeviceIdArray allDeviceIds;
        XsDeviceIdArray mtwDeviceIds;
        XsDevicePtrArray mtwDevices;

        // VQF
        std::vector<std::pair<int, VQF>> vqfContainer;
        std::vector<imu_msgs::msg::IMUData> imu_data_msg;

        // Debugging
        std::chrono::system_clock::time_point start_time;
        std::chrono::system_clock::time_point end_time;
        std::chrono::duration<double, std::milli> elapsed_time;
        int counter_ = 0;
        

        void timerCallback();
        void handleError(std::string error);
        void handleAbort(int);
    };


    /*! \brief Stream insertion operator overload for XsPortInfo */
    std::ostream& operator << (std::ostream& out, XsPortInfo const& p);

    /*! \brief Stream insertion operator overload for XsDevice */
    std::ostream& operator << (std::ostream& out, XsDevice const& d);

} /* namespace xsens_mtw_manager */