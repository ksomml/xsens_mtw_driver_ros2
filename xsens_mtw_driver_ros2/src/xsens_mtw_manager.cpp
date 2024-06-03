#include "../include/xsens_mtw_driver_ros2/xsens_mtw_manager.hpp"

namespace xsens_mtw_manager
{
    XsensManager::XsensManager(const std::string& name)
        : Node(name)
    {
        // get main loop rate
        this->declare_parameter("rate", 200.0);
        rate = this->get_parameter("rate").as_double();
        double timestep = (1 / rate) * 1000;
        auto rate_ms = std::chrono::milliseconds(int(timestep));
        timer_ = this->create_wall_timer(rate_ms, std::bind(&XsensManager::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Publish rate set to: %.2f Hz", rate);

        // get IMU rate
        this->declare_parameter("imu_rate", 100.0);
        imu_rate = this->get_parameter("imu_rate").as_double();

        const int desiredUpdateRate = static_cast<int>(imu_rate);
        const int desiredRadioChannel = 25;		// Use radio channel 25 for wireless master. (try channels 11, 15, 20 or 25)

        RCLCPP_INFO(this->get_logger(), "Creating XsControl object...");
        control = XsControl::construct();
        if (control == 0)
        {
            handleError("Failed to construct XsControl instance.");
            return;
        }

        detectedDevices = XsScanner::scanPorts();
        wirelessMasterPort = detectedDevices.begin();

        RCLCPP_INFO(this->get_logger(), "Scanning for devices...");

        while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
        {
            ++wirelessMasterPort;
        }
        if (wirelessMasterPort == detectedDevices.end())
        {
            throw std::runtime_error("No wireless masters found");
        }


        RCLCPP_INFO(this->get_logger(), "Found a device with ID: %s @ port: %s, baudrate: %d",
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

        RCLCPP_INFO(this->get_logger(), "Getting the list of the supported update rates...");
        const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

        RCLCPP_INFO_STREAM(this->get_logger(), "Supported update rates: ");
        for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
        {
            std::cout << *itUpRate << " ";
        }
        std::cout << std::endl;

        const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);
        const double timer_period = 1.0 / static_cast<double>(newUpdateRate);
        updaterate_debug = newUpdateRate;

        RCLCPP_INFO_STREAM(this->get_logger(), "Setting update rate to " << newUpdateRate << " Hz...");
        if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
        {
            std::ostringstream error;
            error << "Failed to set update rate: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

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

        RCLCPP_INFO_STREAM(this->get_logger(), "Setting radio channel to " << desiredRadioChannel << " and enabling radio...");
        if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
        {
            std::ostringstream error;
            error << "Failed to set radio channel: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for MTW to wirelessly connect... Press 'y' to start measurement or 'q' to end node.");
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
                    RCLCPP_INFO_STREAM(this->get_logger(), "Number of connected MTWs: " << (int)nextCount);
                    connectedMTWCount = nextCount;
                }
                else
                {
                    break;
                }
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

        if (interruption) cleanupAndShutdown();

        RCLCPP_INFO(this->get_logger(), "Getting XsDevice instances for all MTWs...");
        allDeviceIds = control->deviceIds();

        for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
        {
            if (i->isMtw())
            {
                mtwDeviceIds.push_back(*i);
            }
        }

        for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
        {
            XsDevicePtr mtwDevice = control->device(*i);
            if (mtwDevice != 0)
            {
                mtwDevices.push_back(mtwDevice);
            }
            else
            {
                throw std::runtime_error("Failed to create an MTW XsDevice instance");
            }
        }

        RCLCPP_INFO(this->get_logger(), "Attaching callback handlers to MTWs...");
        mtwCallbacks.resize(mtwDevices.size());
        for (int i = 0; i < (int)mtwDevices.size(); ++i)
        {
            mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
            mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
        }


        RCLCPP_INFO(this->get_logger(), "VQF timer period set: %.4fs (rate: %dHz)", timer_period, newUpdateRate);

        // VQF
        for (int i = 0; i < (int)mtwDevices.size(); ++i)
        {
            VQF vqf(timer_period);
            vqfContainer.emplace_back(i, vqf);
        }

        // Setup message
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

        RCLCPP_INFO(this->get_logger(), "Starting measurement...");
        if (!wirelessMasterDevice->gotoMeasurement())
        {
            std::ostringstream error;
            error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }


        // Publisher
        RCLCPP_INFO(this->get_logger(), "Publish loop starting...");
        imu_pub = this->create_publisher<imu_msgs::msg::IMUDataArray>("xsens_imu_data", 10);
        RCLCPP_INFO(this->get_logger(), "Publishers started, press 's' to stop!");


        // Start timer
        time_start = rclcpp::Clock().now();

        start_time = std::chrono::system_clock::now();
    }

    XsensManager::~XsensManager() {
        cleanupAndShutdown();
    }

    void XsensManager::timer_callback()
    {

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
                        if (i == 0)
                        {
                            counter_++;

                            if (counter_ == updaterate_debug) {
                                RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------------");
                                RCLCPP_INFO(this->get_logger(), "Time elapsed: %.2f s", elapsed_time.count() / 1000);
    
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

                imu_data_array_msg.imu_data.push_back(imu_data_msg[i]);
            }

            auto time_elapsed = (rclcpp::Clock().now() - time_start).seconds();
            imu_data_array_msg.timestamp = time_elapsed;
            imu_pub->publish(imu_data_array_msg);
        }
        else if ((char)_getch() == 's') cleanupAndShutdown();
    }


    void XsensManager::cleanupAndShutdown()
    {
        RCLCPP_INFO(this->get_logger(), "Stopping measurement...");
        wirelessMasterDevice->gotoConfig();

        RCLCPP_INFO(this->get_logger(), "Disabling radio channel...");
        wirelessMasterDevice->disableRadio();

        RCLCPP_INFO(this->get_logger(), "Closing XsControl...");
        control->close();

        RCLCPP_INFO(this->get_logger(), "Deleting MTW callbacks...");
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