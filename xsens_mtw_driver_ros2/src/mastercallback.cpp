#include "../include/xsens_mtw_driver_ros2/mastercallback.hpp"

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------


XsDeviceSet WirelessMasterCallback::getWirelessMTWs() const
{
	XsMutexLocker lock(m_mutex);
	return m_connectedMTWs;
}

void WirelessMasterCallback::onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
{
	XsMutexLocker lock(m_mutex);
	switch (newState)
	{
	case XCS_Disconnected:		/*!< Device has disconnected, only limited informational functionality is available. */

		//std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
		// RCLCPP_INFO_STREAM(node->get_logger(),"EVENT: MTW Disconnected -> " << dev->deviceId().toString().toStdString() );
		m_connectedMTWs.erase(dev);
		break;
	case XCS_Rejected:			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
		// RCLCPP_INFO_STREAM(node->get_logger(),"EVENT: MTW Rejected -> " << dev->deviceId().toString().toStdString() );
		m_connectedMTWs.erase(dev);
		break;
	case XCS_PluggedIn:			/*!< Device is connected through a cable. */
		// RCLCPP_INFO_STREAM(node->get_logger(),"EVENT: MTW PluggedIn -> " << dev->deviceId().toString().toStdString() );
		m_connectedMTWs.erase(dev);
		break;
	case XCS_Wireless:			/*!< Device is connected wirelessly. */
		// RCLCPP_INFO_STREAM(node->get_logger(),"EVENT: MTW Connected -> " << dev->deviceId().toString().toStdString() );
		m_connectedMTWs.insert(dev);
		break;
	case XCS_File:				/*!< Device is reading from a file. */
		// RCLCPP_INFO_STREAM(node->get_logger(),"EVENT: MTW File -> " << dev->deviceId().toString().toStdString() );
		m_connectedMTWs.erase(dev);
		break;
	case XCS_Unknown:			/*!< Device is in an unknown state. */
		// RCLCPP_INFO_STREAM(node->get_logger(),"EVENT: MTW Unkown -> " << dev->deviceId().toString().toStdString() );
		m_connectedMTWs.erase(dev);
		break;
	default:
		// RCLCPP_INFO_STREAM(node->get_logger(),"EVENT: MTW Error -> " << dev->deviceId().toString().toStdString() );
		m_connectedMTWs.erase(dev);
		break;
	}
}
