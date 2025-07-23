#include "../include/xsens_mtw_driver/xsens_mtw_manager.hpp"


namespace xsens_mtw_manager
{
XsensManager::XsensManager(const std::string & name)
    : Node(name)
    , m_status(NO_CONNECTION)
    , m_waitForConnections(true)
    , m_keyInterrupt(false)
    , m_isHeaderWritten(false)
    , m_restartRequested(false)
{
    // --------------------------------------------------------------------
    // ROS2 PARAMETERS
    this->declare_parameter("one_topic_per_imu", true);
    m_oneTopicPerImu = this->get_parameter("one_topic_per_imu").as_bool();

    this->declare_parameter("topic_name", "xsens_imu_data");
    m_topicName = this->get_parameter("topic_name").as_string();

    this->declare_parameter("ros2_rate", 100);
    m_ros2Rate = this->get_parameter("ros2_rate").as_int();

    this->declare_parameter("imu_rate", 100);
    m_imuRate = this->get_parameter("imu_rate").as_int();

    this->declare_parameter("radio_channel", 25);
    m_radioChannel = this->get_parameter("radio_channel").as_int();

    this->declare_parameter("imu_reset_on_record", true);
    m_imuResetOnRecord = this->get_parameter("imu_reset_on_record").as_bool();

    this->declare_parameter("use_magnetometer", false);
    m_useMagnetometer = this->get_parameter("use_magnetometer").as_bool();

    RCLCPP_INFO(this->get_logger(), "Parameters loaded");

    // --------------------------------------------------------------------
    // ROS2 SERVICES
    m_getReadyService = this->create_service<xsrvs::Trigger>(name + "/get_ready",
        std::bind(&XsensManager::getReadyCallback, this, _1, _2));
    m_statusService = this->create_service<xsrvs::Trigger>(name + "/status",
        std::bind(&XsensManager::statusCallback, this, _1, _2));
    m_startService = this->create_service<xsrvs::StartRecording>(name + "/start_recording",
        std::bind(&XsensManager::startRecordingCallback, this, _1, _2));
    m_stopService = this->create_service<xsrvs::Trigger>(name + "/stop_recording",
        std::bind(&XsensManager::stopRecordingCallback, this, _1, _2));
    m_imuResetService = this->create_service<xsrvs::Trigger>(name + "/imu_reset",
        std::bind(&XsensManager::imuResetCallback, this, _1, _2));
    m_restartService = this->create_service<xsrvs::Trigger>(name + "/restart",
        std::bind(&XsensManager::restartDriverCallback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Services started");

    // --------------------------------------------------------------------
    // ROS2 TIMER
    m_connectTimer = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&XsensManager::connectMTWsCallback, this));

    double timestep = (1.0 / m_ros2Rate);
    auto rate_ms = std::chrono::milliseconds(static_cast<int>(timestep*1000));
    m_publishTimer = this->create_wall_timer(rate_ms,
        std::bind(&XsensManager::publishDataCallback, this));
    m_publishTimer->cancel();    // will be started when status switches to READY

    RCLCPP_INFO(this->get_logger(), "ROS2 publish rate: %d Hz", m_ros2Rate);

    // --------------------------------------------------------------------
    // FILE
    m_file.precision(16);  // sets the precision of the floating-point values
    m_file << std::fixed;  // disables scientific notation

    // --------------------------------------------------------------------
    // INITIAL MASTER SETUP
    try
    {
        initialMasterSetup();
    }
    catch (std::runtime_error const & e)
    {
        logAndRaiseSigInt(e.what());
    }
}

XsensManager::~XsensManager()
{
    cleanupAndShutdown();
}

// Initialize the Xsens Master device and start the connectMTWsCallback() loop afterwards
void XsensManager::initialMasterSetup()
{
    // --------------------------------------------------------------------
    // XSENS API
    RCLCPP_INFO(this->get_logger(), "Creating XsControl object...");
    m_control = XsControl::construct();
    if (m_control == 0)
    {
        std::ostringstream error;
        error << "Failed to construct XsControl instance.";
        throw std::runtime_error(error.str());
    }

    // --------------------------------------------------------------------
    // DEVICE - MASTER SETUP
    m_waitForConnections = true;
    m_detectedDevices = XsScanner::scanPorts();
    m_wirelessMasterPort = m_detectedDevices.begin();

    RCLCPP_INFO(this->get_logger(), "Scanning for devices...");
    while (m_wirelessMasterPort != m_detectedDevices.end() &&
        !m_wirelessMasterPort->deviceId().isWirelessMaster()) ++m_wirelessMasterPort;
    if (m_wirelessMasterPort == m_detectedDevices.end())
        throw std::invalid_argument("No wireless masters found!");

    RCLCPP_INFO(this->get_logger(), "Found a device with ID: %s @ port: %s, baudrate: %d",
        m_wirelessMasterPort->deviceId().toString().toStdString().c_str(),
        m_wirelessMasterPort->portName().toStdString().c_str(), m_wirelessMasterPort->baudrate());

    RCLCPP_INFO(this->get_logger(), "Opening port...");
    if (!m_control->openPort(m_wirelessMasterPort->portName().toStdString(),
        m_wirelessMasterPort->baudrate()))
    {
        std::ostringstream error;
        error << "Failed to open port " << *m_wirelessMasterPort;
        throw std::runtime_error(error.str());
    }

    RCLCPP_INFO(this->get_logger(), "Creating XsDevice instance for wireless master...");
    m_wirelessMasterDevice = m_control->device(m_wirelessMasterPort->deviceId());
    if (m_wirelessMasterDevice == 0)
    {
        std::ostringstream error;
        error << "Failed to construct XsDevice instance: " << *m_wirelessMasterPort;
        throw std::runtime_error(error.str());
    }

    RCLCPP_INFO(this->get_logger(), "Putting master into configuration mode...");
    if (!m_wirelessMasterDevice->gotoConfig())
    {
        std::ostringstream error;
        error << "Failed to goto config mode: " << *m_wirelessMasterDevice;
        throw std::runtime_error(error.str());
    }

    RCLCPP_INFO(this->get_logger(), "Attaching callback handler for master...");
    m_wirelessMasterDevice->addCallbackHandler(&m_wirelessMasterCallback);

    // --------------------------------------------------------------------
    // DEVICE - MASTER RADIO CHANNEL
    RCLCPP_INFO(this->get_logger(), "Disabling radio channel if previously enabled...");
    if (m_wirelessMasterDevice->isRadioEnabled())
    {
        if (!m_wirelessMasterDevice->disableRadio())
        {
            std::ostringstream error;
            error << "Failed to disable radio channel: " << *m_wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }
    }

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Setting radio channel to " << m_radioChannel << " and enabling radio..."
    );
    if (!m_wirelessMasterDevice->enableRadio(m_radioChannel))
    {
        std::ostringstream error;
        error << "Failed to set radio channel: " << *m_wirelessMasterDevice;
        throw std::runtime_error(error.str());
    }

    // --------------------------------------------------------------------
    // DEVICE - MTW SCAN
    RCLCPP_INFO(this->get_logger(), "Waiting for MTw's to wirelessly connect...");
    RCLCPP_WARN(this->get_logger(), "Press 'y' to start measurement or 'q' to quit.");
    m_connectedMTWCount = m_wirelessMasterCallback.getWirelessMTWs().size();

    // --------------------------------------------------------------------
    // DEVICE STATUS
    m_status = OK;
    RCLCPP_INFO(this->get_logger(), "is initialized and running");

    // connectMTWsCallback() will be called first and when prompted start publishDataCallback()
    m_connectTimer->reset();
}

// Initial loop to connect all MTw's (depends on m_connectTimer)
void XsensManager::connectMTWsCallback()
{
    if (m_waitForConnections && rclcpp::ok())
    {
        // Connect to MTw's if not connected
        size_t nextCount = m_wirelessMasterCallback.getWirelessMTWs().size();
        if (nextCount != m_connectedMTWCount)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Number of connected MTw's: " << (int)nextCount);
            m_connectedMTWCount = nextCount;
        }

        // Check if any key is pressed
        if (_kbhit())
        {
            char keypressed = static_cast<char>(_getch());
            switch (keypressed)
            {
            case 'y':
                if (m_connectedMTWCount == 0)
                    RCLCPP_WARN(
                        this->get_logger(),
                        "No MTw's connected, press 'y' to start measurement or 'q' to quit."
                    );
                else
                {
                    m_keyInterrupt = true;
                    m_waitForConnections = false;
                }
                break;
            case 'q':
                raise(SIGINT);
                break;
            default:
                break;
            }
        }
    }
    else if (!m_waitForConnections && m_keyInterrupt && rclcpp::ok())  // button press
    {
        m_connectTimer->cancel();  // stop connect timer callback
        m_keyInterrupt = false;

        completeInitialization();
    }
    else if (!m_waitForConnections && !m_keyInterrupt && rclcpp::ok())  // 'get_ready' service call
    {
        // completeInitialization() is called within getReadyCallback()
        m_connectTimer->cancel(); // stop connect timer callback
    }
    else if (!rclcpp::ok())
    {
        RCLCPP_WARN(this->get_logger(), "ROS2 is not running. Shutting down...");
        m_connectTimer->cancel();  // stop connect timer callback
    }
    else
    {
        logAndRaiseSigInt("An unknown error has occured. Aborting.");
    }
}

// Complete missing initialization steps after all MTw's are connected
void XsensManager::completeInitialization()
{
    try
    {
        checkRateSupport();
        mtwSetup();
        vqfSetup();
        rosMessagesSetup();
    }
    catch (std::runtime_error const & e)
    {
        logAndRaiseSigInt(e.what());
    }

    // Adjust size of m_dataTracker
    m_dataTracker.resize(m_connectedMTWCount);

    // Reset m_dataTracker and data_skip_announced_
    for (size_t i = 0; i < m_connectedMTWCount; ++i) m_dataTracker[i] = 0;

    // Setting the maximum data skip for timeout logging
    m_maxDataSkip = static_cast<int>(m_ros2Rate / 10);
    RCLCPP_INFO_STREAM(this->get_logger(), "Max lost packages for timeout log: " << m_maxDataSkip);

    // Check if ROS2 is still running
    if (!rclcpp::ok())
    {
        RCLCPP_WARN(this->get_logger(), "ROS2 is not running. Shutting down...");
        return;
    }

    // ROS2 Publisher
    if (m_oneTopicPerImu){
        // One-topic-per-imu
        for (size_t i = 0; i < m_connectedMTWCount; ++i)
        {
            std::string mtwID = m_mtwDeviceIds[i].toString().toStdString();
			auto imu_pub = this->create_publisher<imu_msgs::msg::IMUDataSingle>(m_topicName + mtwID, 10);
			m_imuPublishers.push_back(imu_pub);
		}
    } else {
        // One-topic-for-all
        m_imuPublisher = this->create_publisher<imu_msgs::msg::IMUDataArray>(m_topicName, 10);
    }

    RCLCPP_WARN(this->get_logger(), "Publishers started, press 'q' to quit");
    RCLCPP_WARN(this->get_logger(), "Press 'r' to start and 's' to stop recording");

    // Set status to READY
    m_status = READY;

    // Reset restart flag
    m_restartRequested = false;


    m_publishTimer->reset();

}

// Main loop to publish IMU data collected from the Xsens MTw's (depends on m_publishTimer)
void XsensManager::publishDataCallback()
{
    imu_msgs::msg::IMUDataArray imu_data_array_msg;

    for (size_t i = 0; i < m_connectedMTWCount; ++i)
    {
        if (m_mtwCallbacks[i]->dataAvailable())
        {
            XsDataPacket const * packet = m_mtwCallbacks[i]->getOldestPacket();

            if (packet->containsCalibratedData())
            {
                // Extract calibrated data from packet
                vqf_real_t acc[3];
                vqf_real_t gyr[3];
                vqf_real_t mag[3];

                acc[0] = packet->calibratedAcceleration().value(0);
                acc[1] = packet->calibratedAcceleration().value(1);
                acc[2] = packet->calibratedAcceleration().value(2);

                gyr[0] = packet->calibratedGyroscopeData().value(0);
                gyr[1] = packet->calibratedGyroscopeData().value(1);
                gyr[2] = packet->calibratedGyroscopeData().value(2);

                mag[0] = packet->calibratedMagneticField().value(0);
                mag[1] = packet->calibratedMagneticField().value(1);
                mag[2] = packet->calibratedMagneticField().value(2);

                // Update message data with new sensor readings
                m_imuDataMsg[i].id = m_mtwDeviceIds[i].toString().toStdString();
                m_imuDataMsg[i].angular_velocity.x = gyr[0];  // [rad/s]
                m_imuDataMsg[i].angular_velocity.y = gyr[1];  // [rad/s]
                m_imuDataMsg[i].angular_velocity.z = gyr[2];  // [rad/s]
                m_imuDataMsg[i].linear_acceleration.x = acc[0];  // [m/s²]
                m_imuDataMsg[i].linear_acceleration.y = acc[1];  // [m/s²]
                m_imuDataMsg[i].linear_acceleration.z = acc[2];  // [m/s²]
                m_imuDataMsg[i].magnetic_field.x = mag[0];  // [uT]
                m_imuDataMsg[i].magnetic_field.y = mag[1];  // [uT]
                m_imuDataMsg[i].magnetic_field.z = mag[2];  // [uT]

                // Reset m_dataTracker if new data is available
                m_dataTracker[i] = 0;
            }

            m_mtwCallbacks[i]->deleteOldestPacket();
        }
        else m_dataTracker[i]++;

        // Always update VQF filter with the latest data (either new or last known)
        vqf_real_t quat[4];
        vqf_real_t acc[3] = {
            m_imuDataMsg[i].linear_acceleration.x,
            m_imuDataMsg[i].linear_acceleration.y,
            m_imuDataMsg[i].linear_acceleration.z
        };
        vqf_real_t gyr[3] = {
            m_imuDataMsg[i].angular_velocity.x,
            m_imuDataMsg[i].angular_velocity.y,
            m_imuDataMsg[i].angular_velocity.z
        };
        vqf_real_t mag[3] = {
            m_imuDataMsg[i].magnetic_field.x,
            m_imuDataMsg[i].magnetic_field.y,
            m_imuDataMsg[i].magnetic_field.z
        };

        // Perform a filter update step for one sample to get the new orientation
        if (!m_useMagnetometer)
        {
            m_vqfFilters[i].update(gyr, acc);
            m_vqfFilters[i].getQuat6D(quat);
        }
        else
        {
            m_vqfFilters[i].update(gyr, acc, mag);
            m_vqfFilters[i].getQuat9D(quat);
        }m_timestamp = this->now().nanoseconds();

        // Update orientation quaternion in message
        m_imuDataMsg[i].orientation.w = quat[0];
        m_imuDataMsg[i].orientation.x = quat[1];
        m_imuDataMsg[i].orientation.y = quat[2];
        m_imuDataMsg[i].orientation.z = quat[3];

        // Using last known data if no new data is available
        imu_data_array_msg.imu_data.push_back(m_imuDataMsg[i]);
        
        // Publish one-topic-per-imu
        if (m_oneTopicPerImu){
            imu_msgs::msg::IMUDataSingle imu_data_single_msg;

            imu_data_single_msg.imu_data = m_imuDataMsg[i];
            
            m_timestamp = this->now().nanoseconds();
            imu_data_single_msg.timestamp = m_timestamp;

            m_imuPublishers[i]->publish(imu_data_single_msg);
        }
        

        // Announce IMU timeout
        if (m_dataTracker[i] > m_maxDataSkip)
        {
            RCLCPP_WARN_STREAM_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,  // ms
                "IMU " << m_imuDataMsg[i].id << " timed out..."
            );
        }
    }

    // Publish IMU data for one-topic-for-all
    if (!m_oneTopicPerImu){
        m_timestamp = this->now().nanoseconds();
        imu_data_array_msg.timestamp = m_timestamp;
        m_imuPublisher->publish(imu_data_array_msg);
    }


    if (m_status == RECORDING) writeDataToFile();

    // Check if a key is pressed
    if (_kbhit())
    {
        char keypressed = static_cast<char>(_getch());
        switch (keypressed)
        {
        case 'q':
            if (m_status == RECORDING) stopRecording();
            raise(SIGINT);
            break;
        case 'r':
            if (m_status == RECORDING) RCLCPP_INFO(this->get_logger(), "Already recording...");
            else
            {
                if (m_imuResetOnRecord) resetIMUs();
                startRecording();
            }
            break;
        case 's':
            if (m_status != RECORDING) RCLCPP_INFO(this->get_logger(), "Currently not recording...");
            else stopRecording();
            break;
        default:
            break;
        }
    }
}

// Check if desired rate is supported for connected IMU's
void XsensManager::checkRateSupport()
{
    int maxUpdateRate = getMaxUpdateRate(static_cast<int>(m_connectedMTWCount));

    if (maxUpdateRate == -1)
    {
        std::ostringstream error;
        error << "Unsupported number of MTw's connected: " << m_connectedMTWCount;
        throw std::runtime_error(error.str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Getting the list of the supported update rates...");
        const XsIntArray supportedUpdateRates = m_wirelessMasterDevice->supportedUpdateRates();

        std::ostringstream supportedUpdateRatesStr;
        for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin();
            itUpRate != supportedUpdateRates.end(); ++itUpRate)
        {
            supportedUpdateRatesStr << *itUpRate << " ";
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Supported update rates: " << supportedUpdateRatesStr.str();
        );

        const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, m_imuRate);
        m_imuRate = newUpdateRate;

        if (m_imuRate > maxUpdateRate)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "%d Hz is not supported for %ld connected MTw's",
                m_imuRate,
                m_connectedMTWCount
            );
            RCLCPP_WARN(
                this->get_logger(),
                "Lowering imu update rate to %d Hz...",
                maxUpdateRate
            );
            m_imuRate = maxUpdateRate;
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "%d Hz is supported for %ld connected MTw's",
                m_imuRate,
                m_connectedMTWCount
            );
        }
    }

    // Setting the update rate
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting IMU update rate: " << m_imuRate << " Hz");
    if (!m_wirelessMasterDevice->setUpdateRate(m_imuRate))
    {
        std::ostringstream error;
        error << "Failed to set update rate: " << *m_wirelessMasterDevice;
        throw std::runtime_error(error.str());
    }
}

// Get the maximum update rate for the connected IMU's following the Xsens documentation
int XsensManager::getMaxUpdateRate(int t_imuSize)
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

    if (t_imuSize >= 1 && t_imuSize <= 5) return 120;
    else if (t_imuSize >= 6 && t_imuSize <= 9) return 100;
    else if (t_imuSize == 10) return 80;
    else if (t_imuSize >= 11 && t_imuSize <= 20) return 60;
    else if (t_imuSize >= 21 && t_imuSize <= 32) return 40;
    else return -1;
}

// Find the closest supported update rate to the user's desired update rate
int XsensManager::findClosestUpdateRate(
    const XsIntArray & t_supportedUpdateRates,
    const int t_desiredUpdateRate
)
{
    if (t_supportedUpdateRates.empty())
    {
        std::ostringstream error;
        error << "No supported update rates found.";
        throw std::runtime_error(error.str());
    }

    if (t_supportedUpdateRates.size() == 1) return t_supportedUpdateRates[0];

    int uRateDist = -1;
    int closestUpdateRate = -1;
    for (XsIntArray::const_iterator itUpRate = t_supportedUpdateRates.begin();
        itUpRate != t_supportedUpdateRates.end(); ++itUpRate)
    {
        const int currDist = std::abs(*itUpRate - t_desiredUpdateRate);

        if ((uRateDist == -1) || (currDist < uRateDist))
        {
            uRateDist = currDist;
            closestUpdateRate = *itUpRate;
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Closest supported update rate: %d Hz (desired: %d Hz)",
        closestUpdateRate,
        m_imuRate
    );

    return closestUpdateRate;
}

// Put the master device into measurement mode and get the XsDevice instances for all MTw's
void XsensManager::mtwSetup()
{
    RCLCPP_INFO(this->get_logger(), "Putting device into measurement mode...");
    if (!m_wirelessMasterDevice->gotoMeasurement())
    {
        std::ostringstream error;
        error << "Could not put device " << *m_wirelessMasterDevice << " into measurement mode.";
        throw std::runtime_error(error.str());
    }

    RCLCPP_INFO(this->get_logger(), "Getting XsDevice instances for all MTw's...");
    m_allDeviceIds = m_control->deviceIds();

    for (XsDeviceIdArray::const_iterator i = m_allDeviceIds.begin(); i != m_allDeviceIds.end(); ++i)
    {
        if (i->isMtw()) m_mtwDeviceIds.push_back(*i);
    }

    for (XsDeviceIdArray::const_iterator i = m_mtwDeviceIds.begin(); i != m_mtwDeviceIds.end(); ++i)
    {
        XsDevicePtr mtwDevice = m_control->device(*i);
        if (mtwDevice != 0) m_mtwDevices.push_back(mtwDevice);
        else throw std::runtime_error("Failed to create an MTw XsDevice instance");
    }

    RCLCPP_INFO(this->get_logger(), "Attaching callback handlers to MTw's...");
    m_mtwCallbacks.resize(m_connectedMTWCount);
    for (int i = 0; i < static_cast<int>(m_connectedMTWCount); ++i)
    {
        m_mtwCallbacks[i] = new MtwCallback(i, m_mtwDevices[i]);
        m_mtwDevices[i]->addCallbackHandler(m_mtwCallbacks[i]);
    }
}

// Setup a VQF instance for each connected MTw
void XsensManager::vqfSetup()
{
    // VQF - Quaternion Filter (https://vqf.readthedocs.io/en/latest/)
    const double timer_period = 1.0 / m_imuRate;

    // Magnetometer parameters
    VQFParams vqf_params;
    vqf_params.tauMag = 9.0;                    // [s]      (default: 9.0)
    vqf_params.magDistRejectionEnabled = true;  //          (default: true)
    vqf_params.magCurrentTau = 0.05;            // [s]      (default: 0.05)
    vqf_params.magRefTau = 20.0;                // [s]      (default: 20.0)
    vqf_params.magNormTh = 0.1;                 // [10%]    (default: 0.1)
    vqf_params.magNewTime = 20.0;               // [s]      (default: 20.0)
    vqf_params.magNewFirstTime = 5.0;           // [s]      (default: 5.0)
    vqf_params.magNewMinGyr = 20.0;             // [deg/s]  (default: 20.0)
    vqf_params.magMinUndisturbedTime = 0.5;     // [s]      (default: 0.5)
    vqf_params.magMaxRejectionTime = 60.0;      // [s]      (default: 60.0)
    vqf_params.magRejectionFactor = 2.0;        //          (default: 2.0)

    for (int i = 0; i < static_cast<int>(m_connectedMTWCount); ++i)
    {
        VQF vqf(vqf_params, timer_period);
        m_vqfFilters.push_back(vqf);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "VQF timer period: %.4fs (rate: %dHz)",
        timer_period,
        m_imuRate
    );

    if (m_useMagnetometer)
        RCLCPP_INFO(this->get_logger(), "VQF: Magnetometer enabled");
    else
        RCLCPP_INFO(this->get_logger(), "VQF: Magnetometer disabled");
}

// Setup the ROS2 messages for the IMU data filled with default values
void XsensManager::rosMessagesSetup()
{
    m_imuDataMsg.resize(m_connectedMTWCount);

    for (size_t i = 0; i < m_connectedMTWCount; ++i)
    {
        // toString(): Returns a human readable XsString of the XsDeviceId
        // toStdString(): Converts the XsString to a std::string needed for the ROS2 message
        m_imuDataMsg[i].id = m_mtwDeviceIds[i].toString().toStdString();
        m_imuDataMsg[i].orientation.w = 1.0;
        m_imuDataMsg[i].orientation.x = 0.0;
        m_imuDataMsg[i].orientation.y = 0.0;
        m_imuDataMsg[i].orientation.z = 0.0;
        m_imuDataMsg[i].angular_velocity.x = 0.0;
        m_imuDataMsg[i].angular_velocity.y = 0.0;
        m_imuDataMsg[i].angular_velocity.z = 0.0;
        m_imuDataMsg[i].linear_acceleration.x = 0.0;
        m_imuDataMsg[i].linear_acceleration.y = 0.0;
        m_imuDataMsg[i].linear_acceleration.z = 0.0;
        m_imuDataMsg[i].magnetic_field.x = 0.0;
        m_imuDataMsg[i].magnetic_field.y = 0.0;
        m_imuDataMsg[i].magnetic_field.z = 0.0;
    }
}

// Generate and set a file name based on the current date and time
void XsensManager::generateFileName()
{
    std::time_t t = std::time(0);
    std::tm * now = std::localtime(&t);

    std::ostringstream oss;
    oss << "xsens_imu_data_" << (now->tm_year + 1900) << "-" << (now->tm_mon + 1) << "-"
        << now->tm_mday << "_" << now->tm_hour << "-" << now->tm_min << "-" << now->tm_sec
        << ".csv";
    m_fileName = oss.str();
}

// Write the header to the file in the converted rosbag to csv format
void XsensManager::writeFileHeader()
{
    // First line
    m_file << "IMUDataArray,";
    for (size_t i = 0; i < m_connectedMTWCount; i++)
    {
        // match the number of 13 needed columns per device
        for (int j = 0; j < 14; j++)
        {
            m_file << "imu_data[" << i << "]";
            if (i != m_connectedMTWCount - 1 || j != 13) m_file << ",";
        }
    }
    m_file << "\n";

    // Second line
    m_file << "timestamp,";
    for (size_t i = 0; i < m_connectedMTWCount; ++i)
    {
        m_file << "id,";
        m_file << "orientation,orientation,orientation,orientation,";
        m_file << "angular_velocity,angular_velocity,angular_velocity,";
        m_file << "linear_acceleration,linear_acceleration,linear_acceleration,";
        m_file << "magnetic_field,magnetic_field,magnetic_field";

        if (i != m_connectedMTWCount - 1) m_file << ",";
        else m_file << "\n";
    }

    // Third line
    m_file << "nan,";
    for (size_t i = 0; i < m_connectedMTWCount; ++i)
    {
        m_file << "nan,";
        m_file << "w,x,y,z,";
        m_file << "x,y,z,";
        m_file << "x,y,z,";
        m_file << "x,y,z";

        if (i != m_connectedMTWCount - 1) m_file << ",";
        else m_file << "\n";
    }
}

// Write the IMU data to a csv file with multi-indexing (3 line header)
void XsensManager::writeDataToFile()
{
    m_file << m_timestamp << ",";

    for (size_t i = 0; i < m_connectedMTWCount; ++i)
    {
        m_file << m_imuDataMsg[i].id << ",";
        m_file << m_imuDataMsg[i].orientation.w << ",";
        m_file << m_imuDataMsg[i].orientation.x << ",";
        m_file << m_imuDataMsg[i].orientation.y << ",";
        m_file << m_imuDataMsg[i].orientation.z << ",";
        m_file << m_imuDataMsg[i].angular_velocity.x << ",";
        m_file << m_imuDataMsg[i].angular_velocity.y << ",";
        m_file << m_imuDataMsg[i].angular_velocity.z << ",";
        m_file << m_imuDataMsg[i].linear_acceleration.x << ",";
        m_file << m_imuDataMsg[i].linear_acceleration.y << ",";
        m_file << m_imuDataMsg[i].linear_acceleration.z << ",";
        m_file << m_imuDataMsg[i].magnetic_field.x << ",";
        m_file << m_imuDataMsg[i].magnetic_field.y << ",";
        m_file << m_imuDataMsg[i].magnetic_field.z;

        if (i != m_connectedMTWCount - 1) m_file << ",";
        else m_file << "\n";
    }
}

// Close the file if it is open
void XsensManager::closeFile()
{
    if (m_file.is_open())
    {
        RCLCPP_INFO(this->get_logger(), "Closing file...");
        m_file.close();
    }
}

// Generate a csv file and start recording IMU data
void XsensManager::startRecording(const std::shared_ptr<xsrvs::StartRecording::Request> t_request)
{
    if (t_request != nullptr)
    {
        // Generate file name based on request
        std::ostringstream oss;
        oss << "xsens_imu_data_" << t_request->id << ".csv";
        m_fileName = oss.str();
    }
    else generateFileName();

    if (!m_file.is_open())
    {
        m_file.open(m_fileName, std::ios::out);
        if (!m_file) throw std::runtime_error("Unable to open file");
    }
    writeFileHeader();

    m_status = RECORDING;
    RCLCPP_WARN(this->get_logger(), "STARTED RECORDING");
}

// Close the file and stop recording IMU data
void XsensManager::stopRecording()
{
    m_status = READY;
    closeFile();
    RCLCPP_WARN(this->get_logger(), "STOPPED RECORDING");
}

// Reset the VQF filters for all connected MTw's
void XsensManager::resetIMUs()
{
    for (auto &vqf : m_vqfFilters)
    {
        vqf.resetState();
    }

    RCLCPP_WARN(this->get_logger(), "IMUs reset");
}

// Service: Return the current status to the service request
void XsensManager::statusCallback(
    const std::shared_ptr<xsrvs::Trigger::Request> t_request,
    std::shared_ptr<xsrvs::Trigger::Response> t_response
)
{
    (void)t_request;

    t_response->hardware_status = m_status;
}

// Service: Get the device in a ready if not ready and return the current status
void XsensManager::getReadyCallback(
    const std::shared_ptr<xsrvs::Trigger::Request> t_request,
    std::shared_ptr<xsrvs::Trigger::Response> t_response
)
{
    (void)t_request;

    switch (m_status)
    {
    case OK:
        if (m_connectedMTWCount == 0)
        {
            t_response->hardware_status = m_status;
            RCLCPP_WARN(this->get_logger(), "No MTw's connected");
            break;
        }
        m_waitForConnections = false;  // stops connectMTWsCallback() callback
        completeInitialization();
        t_response->hardware_status = m_status;
        RCLCPP_WARN(this->get_logger(), "Device is ready");
        break;
    case READY:
        t_response->hardware_status = m_status;
        RCLCPP_WARN(this->get_logger(), "Device is already ready");
        break;
    case NO_CONNECTION:
        t_response->hardware_status = m_status;
        RCLCPP_WARN(this->get_logger(), "Device is still initializing");
        break;
    default:
        t_response->hardware_status = m_status;
        RCLCPP_WARN(this->get_logger(), "Cannot get ready at this moment");
        break;
    }
}

// Service: Start recording IMU data to a csv file if ready and return the current status
void XsensManager::startRecordingCallback(
    const std::shared_ptr<xsrvs::StartRecording::Request> t_request,
    std::shared_ptr<xsrvs::StartRecording::Response> t_response
)
{
    switch (m_status)
    {
    case READY:
        startRecording(t_request);    // includes logging
        t_response->hardware_status = m_status;
        break;
    case RECORDING:
        t_response->hardware_status = m_status;
        RCLCPP_WARN(this->get_logger(), "Already recording");
        break;
    default:
        t_response->hardware_status = m_status;
        RCLCPP_WARN(this->get_logger(), "Device is not ready");
        break;
    }
}

// Service: Stop recording IMU data to a csv file if recording and return the current status
void XsensManager::stopRecordingCallback(
    const std::shared_ptr<xsrvs::Trigger::Request> t_request,
    std::shared_ptr<xsrvs::Trigger::Response> t_response
)
{
    (void)t_request;

    switch (m_status)
    {
    case RECORDING:
        stopRecording();    // includes logging
        t_response->hardware_status = m_status;
        break;
    case READY:
        t_response->hardware_status = m_status;
        RCLCPP_WARN(this->get_logger(), "Currently not recording");
        break;
    default:
        t_response->hardware_status = m_status;
        RCLCPP_WARN(this->get_logger(), "Device is not ready");
        break;
    }
}


// Service: Reset the VQF filters for all connected MTw's and return the current status
void XsensManager::imuResetCallback(const std::shared_ptr<xsrvs::Trigger::Request> t_request,
    std::shared_ptr<xsrvs::Trigger::Response> t_response)
{
    (void)t_request;

    switch (m_status)
    {
    case READY:
        resetIMUs();  // includes logging
        t_response->hardware_status = m_status;
        break;
    case RECORDING:
        resetIMUs();  // includes logging
        t_response->hardware_status = m_status;
        break;
    default:
        t_response->hardware_status = m_status;
        RCLCPP_WARN(this->get_logger(), "No IMUs are initialized yet");
        break;
    }
}

// Service: Restart the Xsens driver and return the NO_CONNECTION status before the restart
void XsensManager::restartDriverCallback(const std::shared_ptr<xsrvs::Trigger::Request> t_request,
    std::shared_ptr<xsrvs::Trigger::Response> t_response)
{
    (void)t_request;

    RCLCPP_WARN(this->get_logger(), "Restarting driver...");

    if (m_status == RECORDING) stopRecording();
    m_status = NO_CONNECTION;
    t_response->hardware_status = m_status;

    // Stop publish timer and set restart flag
    m_publishTimer->cancel();
    m_restartRequested = true;

    // Reinitialize
    try
    {
        cleanupAndShutdown();
        initialMasterSetup();
    }
    catch (std::runtime_error const & e)
    {
        logAndRaiseSigInt(e.what());
    }
}

// Cleanup all variables, optionally turn off MTw's and shutdown the master device
void XsensManager::cleanupAndShutdown()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up and shutting down...");

    // Close file_ if open
    closeFile();

    // Cleanup VQF containers
    if (!m_vqfFilters.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up VQF instances...");
        m_vqfFilters.clear();
    }

    // Cleanup Xsens API
    RCLCPP_INFO(this->get_logger(), "Putting master into configuration mode...");
    if (!m_wirelessMasterDevice->gotoConfig())
    {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Failed to put device " << *m_wirelessMasterDevice << " into configuration mode."
        );
    }

    // Wait for button press to optionally turn off MTw's if device status is ready
    if (m_status == READY)
    {
        RCLCPP_WARN(this->get_logger(), "Turn off MTw's? (y/n)");
        bool interruption = false;
        int timeout_counter = 0;  // 100 * 200ms = 20s timeout

        while (!interruption && (timeout_counter < 100))
        {
            // Check keypresses
            if (_kbhit())
            {
                char keypressed = static_cast<char>(_getch());
                std::ostringstream mtw_success, mtw_fail;

                switch (keypressed)
                {
                case 'y':
                    RCLCPP_INFO(this->get_logger(), "Turning off MTw's...");
                    interruption = true;
                    for (int i = 0; i < static_cast<int>(m_connectedMTWCount); ++i)
                    {
                        // Transport mode equals IMU shutdown
                        if (!m_mtwDevices[i]->setTransportMode(true))
                        {
                            mtw_fail << m_mtwDevices[i]->deviceId().toString().toStdString() << " ";
                            RCLCPP_INFO_STREAM(
                                this->get_logger(),
                                "Waiting 15s due to Xsens transport mode cooldown..."
                            );
                            // If transport mode fails, Xsens blocks another request for ~15s
                            XsTime::msleep(15000);
                        }
                        else
                            mtw_success << m_mtwDevices[i]->deviceId().toString().toStdString() << " ";
                    }
                    if (mtw_fail.str().empty())
                        RCLCPP_INFO(this->get_logger(), "All MTw's turned off successfully");
                    else
                    {
                        RCLCPP_INFO_STREAM(
                            this->get_logger(),
                            "MTw's turned off: " << mtw_success.str()
                        );
                        RCLCPP_INFO_STREAM(
                            this->get_logger(),
                            "MTw's failed to turn off: " << mtw_fail.str()
                        );
                    }
                    break;
                case 'n':
                    interruption = true;
                    break;
                }
            }
            XsTime::msleep(200);
            timeout_counter++;
        }

        if (timeout_counter >= 100)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Timeout reached - Shutting down without turning off the MTw's..."
            );
        }
    }

    RCLCPP_INFO(this->get_logger(), "Disabling radio channel...");
    if (!m_wirelessMasterDevice->disableRadio())
    {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Failed to disable radio channel." << *m_wirelessMasterDevice
        );
    }

    RCLCPP_INFO(this->get_logger(), "Closing XsControl...");
    m_control->close();

    if (m_connectedMTWCount > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Deleting MTw callbacks...");
        for (std::vector<MtwCallback *>::iterator i = m_mtwCallbacks.begin();
            i != m_mtwCallbacks.end(); ++i)
        {
            delete (*i);
        }

        m_mtwCallbacks.clear();
    }

    if (m_restartRequested)
    {
        // Cleaning up all to me known variables left to be cleaned for a restart
        // Possible memory leak here due to additional variables that I may not have identified yet
        m_control->destruct();
        m_wirelessMasterDevice = nullptr;
        m_wirelessMasterPort = m_detectedDevices.end();
        m_detectedDevices.clear();
        m_allDeviceIds.clear();
        m_mtwDeviceIds.clear();
        m_mtwDevices.clear();
    }
    else RCLCPP_FATAL(this->get_logger(), "Exited successfully");
}

// Used to trigger cleanup schedule properly
void XsensManager::logAndRaiseSigInt(std::string t_error)
{
    RCLCPP_ERROR(this->get_logger(), "%s", t_error.c_str());
    raise(SIGINT);
}

// Stream insertion operator overload for XsPortInfo
std::ostream & operator << (std::ostream & out, XsPortInfo const & p)
{
    out << "Port: " << std::setw(2) << std::right << p.portNumber() << " ("
        << p.portName().toStdString() << ") @ " << std::setw(7) << p.baudrate()
        << " Bd" << ", " << "ID: " << p.deviceId().toString().toStdString();
    return out;
}

// Stream insertion operator overload for XsDevice
std::ostream & operator << (std::ostream & out, XsDevice const & d)
{
    out << "ID: " << d.deviceId().toString().toStdString() << " ("
        << d.productCode().toStdString() << ")";
    return out;
}

} /* namespace xsens_mtw_manager */