#include "../include/xsens_mtw_driver_ros2/xsens_mtw_manager.hpp"

namespace xsens_mtw_manager
{
    XsensManager::XsensManager(const std::string& name)
        : Node(name)
    {
        // --------------------------------------------------------------------
        // ROS2 PARAMETERS
        this->declare_parameter("topic_name", "xsens_imu_data");
        topic_name = this->get_parameter("topic_name").as_string();

        this->declare_parameter("ros2_rate", 200.0);
        ros2_rate = this->get_parameter("ros2_rate").as_double();

        this->declare_parameter("imu_rate", 100);
        imu_rate = this->get_parameter("imu_rate").as_int();

        this->declare_parameter("radio_channel", 25);
        radio_channel = this->get_parameter("radio_channel").as_int();


        // --------------------------------------------------------------------
        // TIMER
        double timestep = (1 / ros2_rate) * 1000;
        auto rate_ms = std::chrono::milliseconds(int(timestep));
        timer_ = this->create_wall_timer(rate_ms, std::bind(&XsensManager::timerCallback, this));

        RCLCPP_WARN(this->get_logger(), "ROS2 publish rate: %.2f Hz", ros2_rate);


        // --------------------------------------------------------------------
        // XSENS API
        RCLCPP_INFO(this->get_logger(), "Creating XsControl object...");
        control = XsControl::construct();
        if (control == 0)
        {
            handleError("Failed to construct XsControl instance.");
            return;
        }


        // --------------------------------------------------------------------
        // DEVICE - MASTER SETUP
        detectedDevices = XsScanner::scanPorts();
        wirelessMasterPort = detectedDevices.begin();

        RCLCPP_INFO(this->get_logger(), "Scanning for devices...");

        while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster()) ++wirelessMasterPort;
        if (wirelessMasterPort == detectedDevices.end()) throw std::invalid_argument("No wireless masters found!");

        RCLCPP_WARN(this->get_logger(), "Found a device with ID: %s @ port: %s, baudrate: %d",
            wirelessMasterPort->deviceId().toString().toStdString().c_str(),
            wirelessMasterPort->portName().toStdString().c_str(), wirelessMasterPort->baudrate());

        RCLCPP_INFO(this->get_logger(), "Opening port...");
        if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
        {
            std::ostringstream error;
            error << "Failed to open port " << *wirelessMasterPort;
            throw std::runtime_error(error.str());
        }

        RCLCPP_INFO(this->get_logger(), "Getting XsDevice instance for wireless master...");
        wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
        if (wirelessMasterDevice == 0)
        {
            std::ostringstream error;
            error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
            throw std::runtime_error(error.str());
        }
        RCLCPP_INFO(this->get_logger(), "XsDevice instance for master created");

        RCLCPP_INFO(this->get_logger(), "Setting config mode...");
        if (!wirelessMasterDevice->gotoConfig())
        {
            std::ostringstream error;
            error << "Failed to goto config mode: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        RCLCPP_INFO(this->get_logger(), "Attaching callback handler for master...");
        wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);


        // --------------------------------------------------------------------
        // DEVICE - IMU UPDATE RATE
        RCLCPP_INFO(this->get_logger(), "Getting the list of the supported update rates...");
        const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

        std::ostringstream supportedUpdateRatesStr;
        for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
        {
            supportedUpdateRatesStr << *itUpRate << " ";
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Supported update rates: " << supportedUpdateRatesStr.str(););

        const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, imu_rate);
        const double timer_period = 1.0 / static_cast<double>(newUpdateRate);
        imu_rate = newUpdateRate;   // Debugging

        RCLCPP_WARN_STREAM(this->get_logger(), "IMU update rate: " << newUpdateRate << " Hz...");
        if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
        {
            std::ostringstream error;
            error << "Failed to set update rate: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }


        // --------------------------------------------------------------------
        // DEVICE - MASTER RADIO CHANNEL
        RCLCPP_INFO(this->get_logger(), "Disabling radio channel if previously enabled...");
        if (wirelessMasterDevice->isRadioEnabled())
        {
            if (!wirelessMasterDevice->disableRadio())
            {
                std::ostringstream error;
                error << "Failed to disable radio channel: " << *wirelessMasterDevice;
                throw std::runtime_error(error.str());
            }
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Setting radio channel to " << radio_channel << " and enabling radio...");
        if (!wirelessMasterDevice->enableRadio(radio_channel))
        {
            std::ostringstream error;
            error << "Failed to set radio channel: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }


        // --------------------------------------------------------------------
        // DEVICE - MTW SCAN
        RCLCPP_INFO(this->get_logger(), "Waiting for MTw's to wirelessly connect...");
        RCLCPP_WARN(this->get_logger(), "Press 'y' to start measurement or 'q' to quit.");
        bool waitForConnections = true;
        bool interruption = false;
        connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();

        do
        {
            XsTime::msleep(100);

            while (true)
            {
                size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
                if (nextCount != connectedMTWCount)
                {   
                    RCLCPP_INFO_STREAM(this->get_logger(), "Number of connected MTw's: " << (int)nextCount);
                    connectedMTWCount = nextCount;
                }
                else break;
            }

            if (_kbhit())
            {
                char keypressed = (char)_getch();
                if (keypressed == 'y')
                    waitForConnections = false;
                if (keypressed == 'q')
                {
                    interruption = true;
                    waitForConnections = false;
                }
            }

        } while (waitForConnections && rclcpp::ok());

        if (interruption) delete this;      // destructor call


        // --------------------------------------------------------------------
        // DEVICE - MTW SETUP
        RCLCPP_INFO(this->get_logger(), "Putting device into measurement mode...");
        if (!wirelessMasterDevice->gotoMeasurement())
        {
            std::ostringstream error;
            error << "Could not put device " << *wirelessMasterDevice << " into measurement mode.";
            throw std::runtime_error(error.str());
        }

        RCLCPP_INFO(this->get_logger(), "Getting XsDevice instances for all MTw's...");
        allDeviceIds = control->deviceIds();

        for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
            if (i->isMtw()) mtwDeviceIds.push_back(*i);

        for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
        {
            XsDevicePtr mtwDevice = control->device(*i);
            if (mtwDevice != 0) mtwDevices.push_back(mtwDevice);
            else throw std::runtime_error("Failed to create an MTw XsDevice instance");
        }

        RCLCPP_INFO(this->get_logger(), "Attaching callback handlers to MTw's...");
        mtwCallbacks.resize(mtwDevices.size());
        for (int i = 0; i < (int)mtwDevices.size(); ++i)
        {
            mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
            mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
        }


        // --------------------------------------------------------------------
        // VQF - QUATERNION FILTER (https://vqf.readthedocs.io/en/latest/)
        for (int i = 0; i < (int)mtwDevices.size(); ++i)
        {
            VQF vqf(timer_period);
            vqfContainer.emplace_back(i, vqf);
        }

        RCLCPP_INFO(this->get_logger(), "VQF timer period set: %.4fs (rate: %dHz)", timer_period, newUpdateRate);


        // --------------------------------------------------------------------
        // ROS2 MESSAGE
        imu_data_msg.resize(mtwCallbacks.size());

        for (size_t i = 0; i < mtwCallbacks.size(); ++i)
        {
            imu_data_msg[i].id = mtwDeviceIds[i].toString().toStdString();
            imu_data_msg[i].orientation.w = 1.0;
            imu_data_msg[i].orientation.x = 0.0;
            imu_data_msg[i].orientation.y = 0.0;
            imu_data_msg[i].orientation.z = 0.0;
            imu_data_msg[i].angular_velocity.x = 0.0;
            imu_data_msg[i].angular_velocity.y = 0.0;
            imu_data_msg[i].angular_velocity.z = 0.0;
            imu_data_msg[i].linear_acceleration.x = 0.0;
            imu_data_msg[i].linear_acceleration.y = 0.0;
            imu_data_msg[i].linear_acceleration.z = 0.0;
            imu_data_msg[i].magnetic_field.x = 0.0;
            imu_data_msg[i].magnetic_field.y = 0.0;
            imu_data_msg[i].magnetic_field.z = 0.0;
        }


        // --------------------------------------------------------------------
        // ROS2 PUBLISHER
        imu_pub = this->create_publisher<imu_msgs::msg::IMUDataArray>(topic_name, 10);
        RCLCPP_WARN(this->get_logger(), "Publishers started, press 'q' to quit.");
        RCLCPP_WARN(this->get_logger(), "Press 'r' to start and 's' to stop recording.");


        // --------------------------------------------------------------------
        // TIMER (DEBUGGING)
        time_start = rclcpp::Clock().now();
        start_time = std::chrono::system_clock::now();
    }

    XsensManager::~XsensManager() {
        cleanupAndShutdown();
    }


    void XsensManager::timerCallback()
    {
        // --------------------------------------------------------------------
        // MAIN LOOP
        if (!_kbhit())
        {
            imu_msgs::msg::IMUDataArray imu_data_array_msg;

            for (size_t i = 0; i < mtwCallbacks.size(); ++i)
            {
                if (mtwCallbacks[i]->dataAvailable())
                {
                    XsDataPacket const* packet = mtwCallbacks[i]->getOldestPacket();

                    if (packet->containsCalibratedData())
                    {
                        // Debugging
                        if (i == 0)
                        {
                            counter_++;
                            if (counter_ == imu_rate) {
                                // check if loop time is 1.0s which equals to the desired imu update rate
                                RCLCPP_INFO(this->get_logger(), "IMU 1 - LoopTimeCheck (should be 1.00s): %.2fs", elapsed_time.count() / 1000);

                                // reset counter + elapsed_time
                                counter_ = 0;
                                elapsed_time = std::chrono::duration<double, std::milli>(0);
                            }

                            end_time = std::chrono::system_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                            elapsed_time = elapsed_time + static_cast<decltype(elapsed_time)>(duration);
                            start_time = std::chrono::system_clock::now();
                        }
        
                        // VQF filter for quaternion orientation
                        vqf_real_t acc[3];
                        vqf_real_t gyr[3];
                        vqf_real_t mag[3];
                        vqf_real_t quat[4];

                        acc[0] = packet->calibratedAcceleration().value(0);
                        acc[1] = packet->calibratedAcceleration().value(1);
                        acc[2] = packet->calibratedAcceleration().value(2);

                        gyr[0] = packet->calibratedGyroscopeData().value(0);
                        gyr[1] = packet->calibratedGyroscopeData().value(1);
                        gyr[2] = packet->calibratedGyroscopeData().value(2);

                        mag[0] = packet->calibratedMagneticField().value(0);
                        mag[1] = packet->calibratedMagneticField().value(1);
                        mag[2] = packet->calibratedMagneticField().value(2);

                        vqfContainer[i].second.update(gyr, acc, mag);
                        vqfContainer[i].second.getQuat9D(quat);

                        // Update message data
                        imu_data_msg[i].id = mtwDeviceIds[i].toString().toStdString();
                        imu_data_msg[i].orientation.w = quat[0];
                        imu_data_msg[i].orientation.x = quat[1];
                        imu_data_msg[i].orientation.y = quat[2];
                        imu_data_msg[i].orientation.z = quat[3];
                        imu_data_msg[i].angular_velocity.x = gyr[0];		// [rad/s]
                        imu_data_msg[i].angular_velocity.y = gyr[1];		// [rad/s]
                        imu_data_msg[i].angular_velocity.z = gyr[2];		// [rad/s]
                        imu_data_msg[i].linear_acceleration.x = acc[0];		// [m/s²]
                        imu_data_msg[i].linear_acceleration.y = acc[1];		// [m/s²]
                        imu_data_msg[i].linear_acceleration.z = acc[2];		// [m/s²]
                        imu_data_msg[i].magnetic_field.x = mag[0];			// [uT]
                        imu_data_msg[i].magnetic_field.y = mag[1];			// [uT]
                        imu_data_msg[i].magnetic_field.z = mag[2];			// [uT]
                    }

                    mtwCallbacks[i]->deleteOldestPacket();
                }

                // Using last known data if no new data is available
                imu_data_array_msg.imu_data.push_back(imu_data_msg[i]);
            }

            auto time_elapsed = (rclcpp::Clock().now() - time_start).seconds();
            imu_data_array_msg.timestamp = time_elapsed;
            imu_pub->publish(imu_data_array_msg);
        }
        else
        {
            char keypressed = (char)_getch();
            if (keypressed == 'q') delete this;     // destructor call
            if (keypressed == 'r')
            {
                if (isRecording) RCLCPP_INFO(this->get_logger(), "Already recording...");
                else
                {
                    isRecording = true;
                    RCLCPP_WARN(this->get_logger(), "STARTED RECORDING - TODO");
                }
            }
            if (keypressed == 's')
            {
                if (!isRecording) RCLCPP_INFO(this->get_logger(), "Not recording...");
                else
                {
                    isRecording = false;
                    RCLCPP_WARN(this->get_logger(), "STOPPED RECORDING - TODO");
                }
            }
        }
    }

    void XsensManager::cleanupAndShutdown()
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "Putting device into configuration mode...");
            if (!wirelessMasterDevice->gotoConfig())
            {
                std::ostringstream error;
                error << "Could not put device " << *wirelessMasterDevice << " into configuration mode.";
                throw std::runtime_error(error.str());
            }

            RCLCPP_INFO(this->get_logger(), "Disabling radio channel...");
            if (!wirelessMasterDevice->disableRadio())
            {
                std::ostringstream error;
                error << "Failed to disable radio channel: " << *wirelessMasterDevice;
                throw std::runtime_error(error.str());
            }
        }
        catch (std::runtime_error const& e)
        {
            handleError(e.what());
        }

        RCLCPP_INFO(this->get_logger(), "Closing XsControl...");
        control->close();

        RCLCPP_INFO(this->get_logger(), "Deleting MTw callbacks...");
        for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
        {
            delete (*i);
        }

        RCLCPP_INFO(this->get_logger(), "Shutting down.");
        std::exit(EXIT_SUCCESS);
    }

    void XsensManager::handleError(std::string error)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
    }



    /*! \brief Stream insertion operator overload for XsPortInfo */
    std::ostream& operator << (std::ostream& out, XsPortInfo const& p)
    {
        out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
            << std::setw(7) << p.baudrate() << " Bd"
            << ", " << "ID: " << p.deviceId().toString().toStdString()
            ;
        return out;
    }

    /*! \brief Stream insertion operator overload for XsDevice */
    std::ostream& operator << (std::ostream& out, XsDevice const& d)
    {
        out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
        return out;
    }

} /* namespace xsens_mtw_manager */