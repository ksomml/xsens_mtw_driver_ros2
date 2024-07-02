#include "../include/xsens_mtw_driver_ros2/xsens_mtw_manager.hpp"

namespace xsens_mtw_manager
{
    XSensManager::XSensManager(const std::string& name)
        : Node(name)
        , status_(NO_CONNECTION)
        , waitForConnections_(true)
        , interruption_(false)
        , isHeaderWritten_(false)
    {
        // --------------------------------------------------------------------
        // ROS2 PARAMETERS
        this->declare_parameter("topic_name", "xsens_imu_data");
        topic_name_ = this->get_parameter("topic_name").as_string();

        this->declare_parameter("ros2_rate", 200.0);
        ros2_rate_ = this->get_parameter("ros2_rate").as_double();

        this->declare_parameter("imu_rate", 100);
        imu_rate_ = this->get_parameter("imu_rate").as_int();

        this->declare_parameter("radio_channel", 25);
        radio_channel_ = this->get_parameter("radio_channel").as_int();

        this->declare_parameter("reset_timer_on_record", true);
        resetTimerOnRecord_ = this->get_parameter("reset_timer_on_record").as_bool();

        this->declare_parameter("use_magnetometer", false);
        useMagnetometer_ = this->get_parameter("use_magnetometer").as_bool();

        RCLCPP_INFO(this->get_logger(), "Parameters loaded.");

        // --------------------------------------------------------------------
        // ROS2 SERVICES
        get_ready_service_ = this->create_service<std_srvs::srv::Trigger>(name + "/get_ready",
            std::bind(&XSensManager::getReadyCallback, this, _1, _2));
        status_service_ = this->create_service<std_srvs::srv::Trigger>(name + "/status",
            std::bind(&XSensManager::statusCallback, this, _1, _2));
        start_service_ = this->create_service<std_srvs::srv::Trigger>(name + "/start_recording",
            std::bind(&XSensManager::startRecordingCallback, this, _1, _2));
        stop_service_ = this->create_service<std_srvs::srv::Trigger>(name + "/stop_recording",
            std::bind(&XSensManager::stopRecordingCallback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Services started.");


        // --------------------------------------------------------------------
        // TIMER
        connectTimer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&XSensManager::connectMTWsCallback, this));

        double timestep = (1 / ros2_rate_) * 1000;
        auto rate_ms = std::chrono::milliseconds(int(timestep));
        publishTimer_ = this->create_wall_timer(rate_ms,
            std::bind(&XSensManager::publishDataCallback, this));
        publishTimer_->cancel();    // will be started when status switches to READY

        RCLCPP_WARN(this->get_logger(), "ROS2 publish rate: %.2f Hz", ros2_rate_);


        // --------------------------------------------------------------------
        // XSENS API
        RCLCPP_INFO(this->get_logger(), "Creating XsControl object...");
        control_ = XsControl::construct();
        if (control_ == 0)
        {
            handleError("Failed to construct XsControl instance.");
            return;
        }


        // --------------------------------------------------------------------
        // DEVICE - MASTER SETUP
        detectedDevices_ = XsScanner::scanPorts();
        wirelessMasterPort_ = detectedDevices_.begin();

        RCLCPP_INFO(this->get_logger(), "Scanning for devices...");

        while (wirelessMasterPort_ != detectedDevices_.end() && !wirelessMasterPort_->deviceId().isWirelessMaster()) ++wirelessMasterPort_;
        if (wirelessMasterPort_ == detectedDevices_.end()) throw std::invalid_argument("No wireless masters found!");

        RCLCPP_WARN(this->get_logger(), "Found a device with ID: %s @ port: %s, baudrate: %d",
            wirelessMasterPort_->deviceId().toString().toStdString().c_str(),
            wirelessMasterPort_->portName().toStdString().c_str(), wirelessMasterPort_->baudrate());

        RCLCPP_INFO(this->get_logger(), "Opening port...");
        if (!control_->openPort(wirelessMasterPort_->portName().toStdString(), wirelessMasterPort_->baudrate()))
        {
            std::ostringstream error;
            error << "Failed to open port " << *wirelessMasterPort_;
            throw std::runtime_error(error.str());
        }

        RCLCPP_INFO(this->get_logger(), "Getting XsDevice instance for wireless master...");
        wirelessMasterDevice_ = control_->device(wirelessMasterPort_->deviceId());
        if (wirelessMasterDevice_ == 0)
        {
            std::ostringstream error;
            error << "Failed to construct XsDevice instance: " << *wirelessMasterPort_;
            throw std::runtime_error(error.str());
        }
        RCLCPP_INFO(this->get_logger(), "XsDevice instance for master created");

        RCLCPP_INFO(this->get_logger(), "Setting config mode...");
        if (!wirelessMasterDevice_->gotoConfig())
        {
            std::ostringstream error;
            error << "Failed to goto config mode: " << *wirelessMasterDevice_;
            throw std::runtime_error(error.str());
        }

        RCLCPP_INFO(this->get_logger(), "Attaching callback handler for master...");
        wirelessMasterDevice_->addCallbackHandler(&wirelessMasterCallback_);


        // --------------------------------------------------------------------
        // DEVICE - MASTER RADIO CHANNEL
        RCLCPP_INFO(this->get_logger(), "Disabling radio channel if previously enabled...");
        if (wirelessMasterDevice_->isRadioEnabled())
        {
            if (!wirelessMasterDevice_->disableRadio())
            {
                std::ostringstream error;
                error << "Failed to disable radio channel: " << *wirelessMasterDevice_;
                throw std::runtime_error(error.str());
            }
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Setting radio channel to " << radio_channel_ << " and enabling radio...");
        if (!wirelessMasterDevice_->enableRadio(radio_channel_))
        {
            std::ostringstream error;
            error << "Failed to set radio channel: " << *wirelessMasterDevice_;
            throw std::runtime_error(error.str());
        }


        // --------------------------------------------------------------------
        // DEVICE - MTW SCAN
        RCLCPP_INFO(this->get_logger(), "Waiting for MTw's to wirelessly connect...");
        RCLCPP_WARN(this->get_logger(), "Press 'y' to start measurement or 'q' to quit.");
        connectedMTWCount_ = wirelessMasterCallback_.getWirelessMTWs().size();


        // --------------------------------------------------------------------
        // DEVICE STATUS
        status_ = OK;

        // connectMTWsCallback() will be called first and when prompted will start publishDataCallback()
    }

    XSensManager::~XSensManager() {
        cleanupAndShutdown();
    }

    void XSensManager::connectMTWsCallback()
    {
        if (waitForConnections_ && rclcpp::ok())
        {
            // Connect to MTw's if not connected
            size_t nextCount = wirelessMasterCallback_.getWirelessMTWs().size();
            if (nextCount != connectedMTWCount_)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Number of connected MTw's: " << (int)nextCount);
                connectedMTWCount_ = nextCount;
            }

            // Check if any key is pressed
            if (_kbhit())
            {
                char keypressed = (char)_getch();
                switch (keypressed)
                {
                case 'y':
                    if (connectedMTWCount_ == 0)
                        RCLCPP_WARN(this->get_logger(), "No MTw's connected, press 'y' to start measurement or 'q' to quit.");
                    else
                        waitForConnections_ = false;
                    break;
                case 'q':
                    interruption_ = true;
                    waitForConnections_ = false;
                    break;
                default:
                    break;
                }
            }
        }
        else if (!waitForConnections_ && !interruption_ && rclcpp::ok())
        {
            // Finish initialization
            checkRateSupport();
            mtwSetup();
            vqfSetup();
            rosMessagesSetup();

            // Check if ROS2 is still running
            if (!rclcpp::ok()) delete this;

            // ROS2 Publisher
            imu_pub_ = this->create_publisher<imu_msgs::msg::IMUDataArray>(topic_name_, 10);

            RCLCPP_WARN(this->get_logger(), "Publishers started, press 'q' to quit.");
            RCLCPP_WARN(this->get_logger(), "Press 'r' to start and 's' to stop recording.");

            // Start timer
            time_start_ = rclcpp::Clock().now();
            start_time_ = std::chrono::system_clock::now();

            // Set status to READY
            status_ = READY;

            // Stop connect timer
            connectTimer_->cancel();

            // Start publish timer
            publishTimer_->reset();
        }
        else if (interruption_ || !rclcpp::ok())
        {
            delete this;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "An unknown error has occured. Aborting.");
            delete this;
        }
    }

    void XSensManager::publishDataCallback()
    {
        // MAIN LOOP
        if (!_kbhit())
        {
            imu_msgs::msg::IMUDataArray imu_data_array_msg;

            for (size_t i = 0; i < mtwCallbacks_.size(); ++i)
            {
                if (mtwCallbacks_[i]->dataAvailable())
                {
                    XsDataPacket const* packet = mtwCallbacks_[i]->getOldestPacket();

                    if (packet->containsCalibratedData())
                    {
                        // Debugging
                        if (i == 0)
                        {
                            counter_++;
                            if (counter_ == imu_rate_) {
                                // check if loop time is 1.0s which equals to the desired imu update rate
                                RCLCPP_INFO(this->get_logger(), "IMU 1 - LoopTimeCheck (should be 1.00s): %.2fs", elapsed_time_.count() / 1000);

                                // reset counter + elapsed_time
                                counter_ = 0;
                                elapsed_time_ = std::chrono::duration<double, std::milli>(0);
                            }

                            end_time_ = std::chrono::system_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_ - start_time_).count();
                            elapsed_time_ = elapsed_time_ + static_cast<decltype(elapsed_time_)>(duration);
                            start_time_ = std::chrono::system_clock::now();
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

                        if (useMagnetometer_)
                        {
                            vqfContainer_[i].second.update(gyr, acc, mag);
                            vqfContainer_[i].second.getQuat9D(quat);
                        }
                        else
                        {
                            vqfContainer_[i].second.update(gyr, acc);
                            vqfContainer_[i].second.getQuat6D(quat);
                        }

                        // Update message data
                        imu_data_msg_[i].id = mtwDeviceIds_[i].toString().toStdString();
                        imu_data_msg_[i].orientation.w = quat[0];
                        imu_data_msg_[i].orientation.x = quat[1];
                        imu_data_msg_[i].orientation.y = quat[2];
                        imu_data_msg_[i].orientation.z = quat[3];
                        imu_data_msg_[i].angular_velocity.x = gyr[0];		// [rad/s]
                        imu_data_msg_[i].angular_velocity.y = gyr[1];		// [rad/s]
                        imu_data_msg_[i].angular_velocity.z = gyr[2];		// [rad/s]
                        imu_data_msg_[i].linear_acceleration.x = acc[0];		// [m/s²]
                        imu_data_msg_[i].linear_acceleration.y = acc[1];		// [m/s²]
                        imu_data_msg_[i].linear_acceleration.z = acc[2];		// [m/s²]
                        imu_data_msg_[i].magnetic_field.x = mag[0];			// [uT]
                        imu_data_msg_[i].magnetic_field.y = mag[1];			// [uT]
                        imu_data_msg_[i].magnetic_field.z = mag[2];			// [uT]
                    }

                    mtwCallbacks_[i]->deleteOldestPacket();
                }

                // Using last known data if no new data is available
                imu_data_array_msg.imu_data.push_back(imu_data_msg_[i]);
            }

            time_elapsed_ = (rclcpp::Clock().now() - time_start_).seconds();
            imu_data_array_msg.timestamp = time_elapsed_;
            imu_pub_->publish(imu_data_array_msg);

            if (status_ == RECORDING) writeDataToFile();
        }
        else
        {
            char keypressed = (char)_getch();
            switch (keypressed)
            {
            case 'q':
                if (status_ == RECORDING) stopRecording();
                delete this;
                break;
            case 'r':
                if (status_ == RECORDING) RCLCPP_INFO(this->get_logger(), "Already recording...");
                else startRecording();
                break;
            case 's':
                if (status_ != RECORDING) RCLCPP_INFO(this->get_logger(), "Currently not recording...");
                else stopRecording();
                break;
            default:
                break;
            }
        }
    }

    void XSensManager::checkRateSupport()
    {
        /*
        |   MTw | desiredUpdateRate (max) |
        |-------|-------------------------|
        |   1-5 |           120 Hz        |
        |   6-9 |           100 Hz        |
        |    10 |            80 Hz        |
        | 11-20 |            60 Hz        |
        | 21-32 |            40 Hz        |
        */

        // Check if desired imu rate is supported for connected imu's
        int imuSize = static_cast<int>(wirelessMasterCallback_.getWirelessMTWs().size());
        int maxUpdateRate = getMaxUpdateRate(imuSize);

        if (maxUpdateRate == -1)
        {
            std::ostringstream error;
            error << "Unsupported number of MTw's connected: " << imuSize;
            throw std::runtime_error(error.str());
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Getting the list of the supported update rates...");
            const XsIntArray supportedUpdateRates = wirelessMasterDevice_->supportedUpdateRates();

            std::ostringstream supportedUpdateRatesStr;
            for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
                supportedUpdateRatesStr << *itUpRate << " ";
            RCLCPP_INFO_STREAM(this->get_logger(), "Supported update rates: " << supportedUpdateRatesStr.str(););

            const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, imu_rate_);
            imu_rate_ = newUpdateRate;

            if (imu_rate_ > maxUpdateRate)
            {
                RCLCPP_WARN(this->get_logger(), "%d Hz is not supported for %d connected MTw's", imu_rate_, imuSize);
                RCLCPP_WARN(this->get_logger(), "Lowering imu update rate to %d Hz...", maxUpdateRate);
                imu_rate_ = maxUpdateRate;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "%d Hz is supported for %d connected MTw's", imu_rate_, imuSize);
            }
        }

        // Setting the update rate
        RCLCPP_WARN_STREAM(this->get_logger(), "Setting IMU update rate: " << imu_rate_ << " Hz");
        if (!wirelessMasterDevice_->setUpdateRate(imu_rate_))
        {
            std::ostringstream error;
            error << "Failed to set update rate: " << *wirelessMasterDevice_;
            throw std::runtime_error(error.str());
        }
    }

    int XSensManager::getMaxUpdateRate(int imuSize) {
        if (imuSize >= 1 && imuSize <= 5) return 120;
        else if (imuSize >= 6 && imuSize <= 9) return 100;
        else if (imuSize == 10) return 80;
        else if (imuSize >= 11 && imuSize <= 20) return 60;
        else if (imuSize >= 21 && imuSize <= 32) return 40;
        else return -1;
    }

    int XSensManager::findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
    {
        if (supportedUpdateRates.empty())
        {
            std::ostringstream error;
            error << "No supported update rates found.";
            throw std::runtime_error(error.str());
        }

        if (supportedUpdateRates.size() == 1) return supportedUpdateRates[0];

        int uRateDist = -1;
        int closestUpdateRate = -1;
        for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
        {
            const int currDist = std::abs(*itUpRate - desiredUpdateRate);

            if ((uRateDist == -1) || (currDist < uRateDist))
            {
                uRateDist = currDist;
                closestUpdateRate = *itUpRate;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Closest supported update rate: %d Hz (desired: %d Hz)", closestUpdateRate, imu_rate_);

        return closestUpdateRate;
    }

    void XSensManager::mtwSetup()
    {
        RCLCPP_INFO(this->get_logger(), "Putting device into measurement mode...");
        if (!wirelessMasterDevice_->gotoMeasurement())
        {
            std::ostringstream error;
            error << "Could not put device " << *wirelessMasterDevice_ << " into measurement mode.";
            throw std::runtime_error(error.str());
        }

        RCLCPP_INFO(this->get_logger(), "Getting XsDevice instances for all MTw's...");
        allDeviceIds_ = control_->deviceIds();

        for (XsDeviceIdArray::const_iterator i = allDeviceIds_.begin(); i != allDeviceIds_.end(); ++i)
            if (i->isMtw()) mtwDeviceIds_.push_back(*i);

        for (XsDeviceIdArray::const_iterator i = mtwDeviceIds_.begin(); i != mtwDeviceIds_.end(); ++i)
        {
            XsDevicePtr mtwDevice = control_->device(*i);
            if (mtwDevice != 0) mtwDevices_.push_back(mtwDevice);
            else throw std::runtime_error("Failed to create an MTw XsDevice instance");
        }

        RCLCPP_INFO(this->get_logger(), "Attaching callback handlers to MTw's...");
        mtwCallbacks_.resize(mtwDevices_.size());
        for (int i = 0; i < (int)mtwDevices_.size(); ++i)
        {
            mtwCallbacks_[i] = new MtwCallback(i, mtwDevices_[i]);
            mtwDevices_[i]->addCallbackHandler(mtwCallbacks_[i]);
        }
    }

    void XSensManager::vqfSetup()
    {
        // VQF - Quaternion Filter (https://vqf.readthedocs.io/en/latest/)
        const double timer_period_ = 1.0 / static_cast<double>(imu_rate_);

        for (int i = 0; i < (int)mtwDevices_.size(); ++i)
        {
            VQF vqf(timer_period_);
            vqfContainer_.emplace_back(i, vqf);
        }

        RCLCPP_INFO(this->get_logger(), "VQF timer period set: %.4fs (rate: %dHz)", timer_period_, imu_rate_);
    }

    void XSensManager::rosMessagesSetup()
    {
        imu_data_msg_.resize(mtwCallbacks_.size());

        for (size_t i = 0; i < mtwCallbacks_.size(); ++i)
        {
            imu_data_msg_[i].id = mtwDeviceIds_[i].toString().toStdString();
            imu_data_msg_[i].orientation.w = 1.0;
            imu_data_msg_[i].orientation.x = 0.0;
            imu_data_msg_[i].orientation.y = 0.0;
            imu_data_msg_[i].orientation.z = 0.0;
            imu_data_msg_[i].angular_velocity.x = 0.0;
            imu_data_msg_[i].angular_velocity.y = 0.0;
            imu_data_msg_[i].angular_velocity.z = 0.0;
            imu_data_msg_[i].linear_acceleration.x = 0.0;
            imu_data_msg_[i].linear_acceleration.y = 0.0;
            imu_data_msg_[i].linear_acceleration.z = 0.0;
            imu_data_msg_[i].magnetic_field.x = 0.0;
            imu_data_msg_[i].magnetic_field.y = 0.0;
            imu_data_msg_[i].magnetic_field.z = 0.0;
        }
    }

    void XSensManager::generateFileName()
    {
        std::time_t t = std::time(0);
        std::tm* now = std::localtime(&t);

        std::ostringstream oss;
        oss << "xsens_imu_data_" << (now->tm_year + 1900) << "-" << (now->tm_mon + 1) << "-" << now->tm_mday << "_"
            << now->tm_hour << "-" << now->tm_min << "-" << now->tm_sec << ".txt";
        file_name_ = oss.str();
    }

    void XSensManager::writeFileHeader()
    {
        file_ << "time\t";
        for (auto it = mtwDeviceIds_.begin(); it != mtwDeviceIds_.end(); ++it)
        {
            file_ << it->toString().toStdString();
            file_ << (it != std::prev(mtwDeviceIds_.end()) ? "\t" : "\n");
        }
    }

    void XSensManager::writeDataToFile()
    {
        file_ << time_elapsed_ << "\t";

        for (size_t i = 0; i < mtwCallbacks_.size(); ++i)
        {
            file_ << imu_data_msg_[i].orientation.w << ",";
            file_ << imu_data_msg_[i].orientation.x << ",";
            file_ << imu_data_msg_[i].orientation.y << ",";
            file_ << imu_data_msg_[i].orientation.z;

            if (i != mtwCallbacks_.size() - 1) file_ << "\t";
            else file_ << "\n";
        }
    }

    void XSensManager::statusCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        response->success = true;
        response->message = hardwareStatusToString(status_);
    }

    void XSensManager::getReadyCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        switch (status_)
        {
        case OK:
            if (connectedMTWCount_ == 0)
            {
                response->success = false;
                response->message = "No MTw's connected";
                break;
            }
            waitForConnections_ = false;
            response->success = true;
            response->message = "Device is getting ready";
            break;
        case READY:
            response->success = false;
            response->message = "Device is already ready";
            break;
        case NO_CONNECTION:
            response->success = false;
            response->message = "Device is still initializing";
            break;
        default:
            response->success = false;
            response->message = "Cannot get ready at this moment";
            break;
        }
    }

    void XSensManager::startRecordingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        switch (status_)
        {
        case READY:
            startRecording();
            response->success = true;
            response->message = "Recording started";
            break;
        case RECORDING:
            response->success = false;
            response->message = "Already recording";
            break;
        default:
            response->success = false;
            response->message = "Device is not ready";
            break;
        }
    }

    void XSensManager::stopRecordingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        switch (status_)
        {
        case RECORDING:
            stopRecording();
            response->success = true;
            response->message = "Recording stopped";
            break;
        case READY:
            response->success = false;
            response->message = "Currently not recording";
            break;
        default:
            response->success = false;
            response->message = "Device is not ready";
            break;
        }
    }

    void XSensManager::startRecording()
    {
        generateFileName();
        if (!file_.is_open())
        {
            file_.open(file_name_, std::ios::out);
            if (!file_) throw std::runtime_error("Unable to open file");
        }
        writeFileHeader();

        if (resetTimerOnRecord_) time_start_ = rclcpp::Clock().now();

        status_ = RECORDING;
        RCLCPP_WARN(this->get_logger(), "STARTED RECORDING");
    }

    void XSensManager::stopRecording()
    {
        if (file_.is_open()) file_.close();

        status_ = READY;
        RCLCPP_WARN(this->get_logger(), "STOPPED RECORDING");
    }

    void XSensManager::cleanupAndShutdown()
    {
        RCLCPP_WARN(this->get_logger(), "Cleaning up and shutting down...");
        // close file_ if open
        if (file_.is_open()) file_.close();

        // cleanup
        try
        {
            RCLCPP_INFO(this->get_logger(), "Putting device into configuration mode...");
            if (!wirelessMasterDevice_->gotoConfig())
            {
                std::ostringstream error;
                error << "Could not put device " << *wirelessMasterDevice_ << " into configuration mode.";
                throw std::runtime_error(error.str());
            }

            RCLCPP_INFO(this->get_logger(), "Disabling radio channel...");
            if (!wirelessMasterDevice_->disableRadio())
            {
                std::ostringstream error;
                error << "Failed to disable radio channel: " << *wirelessMasterDevice_;
                throw std::runtime_error(error.str());
            }
        }
        catch (std::runtime_error const& e)
        {
            handleError(e.what());
        }

        RCLCPP_INFO(this->get_logger(), "Closing XsControl...");
        control_->close();

        RCLCPP_INFO(this->get_logger(), "Deleting MTw callbacks...");
        for (std::vector<MtwCallback*>::iterator i = mtwCallbacks_.begin(); i != mtwCallbacks_.end(); ++i)
            delete (*i);

        RCLCPP_INFO(this->get_logger(), "Shutting down.");
        std::exit(EXIT_SUCCESS);
    }

    void XSensManager::handleError(std::string error)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
    }


    std::string hardwareStatusToString(HardwareStatus status) {
        switch (status) {
        case ERROR: return "ERROR";
        case NO_CONNECTION: return "NO_CONNECTION";
        case OK: return "OK";
        case READY: return "READY";
        case RECORDING: return "RECORDING";
        default: return "UNKNOWN_STATUS";
        }
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