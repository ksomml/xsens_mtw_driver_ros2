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

// Driver
#include "mastercallback.hpp"
#include "mtwcallback.hpp"
#include "findClosestUpdateRate.hpp"

#include "../include/xsens/xsensdeviceapi.h" 	// The Xsens device API header 
#include "../include/xsens/xstypes.h"
#include "../include/xsens/xsmutex.h"
#include "../include/xsens/conio.h"				// For non ANSI _kbhit() and _getch()

// VQF
#include "vqf.hpp"

// Custom messages
#include "imu_msgs/msg/imu_data.hpp"
#include "imu_msgs/msg/imu_data_array.hpp"
#include "imu_msgs/msg/quaternion.hpp"


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
        double rate;
        double imu_rate;
        size_t connectedMTWCount;

        // Debugging
        std::chrono::system_clock::time_point start_time;
        std::chrono::system_clock::time_point end_time;
        std::chrono::duration<double, std::milli> elapsed_time;
        int counter_ = 0;
        int updaterate_debug = 0;

        
        
        rclcpp::Time time_start;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<std::pair<int, VQF>> vqfContainer;
        std::vector<imu_msgs::msg::IMUData> imu_data_msg;

        rclcpp::Publisher<imu_msgs::msg::IMUDataArray>::SharedPtr imu_pub;

        WirelessMasterCallback wirelessMasterCallback;							// Callback for wireless master
        std::vector<MtwCallback*> mtwCallbacks;									// Callbacks for mtw devices
        XsControl* control;
        XsPortInfoArray detectedDevices;
        XsPortInfoArray::const_iterator wirelessMasterPort;
        XsDevicePtr wirelessMasterDevice;
        XsDeviceIdArray allDeviceIds;
        XsDeviceIdArray mtwDeviceIds;
        XsDevicePtrArray mtwDevices;

        void timer_callback();
        void handleError(std::string error);
        void handleAbort(int);
    };


    /*! \brief Stream insertion operator overload for XsPortInfo */
    std::ostream& operator << (std::ostream& out, XsPortInfo const& p);

    /*! \brief Stream insertion operator overload for XsDevice */
    std::ostream& operator << (std::ostream& out, XsDevice const& d);

} /* namespace xsens_mtw_manager */