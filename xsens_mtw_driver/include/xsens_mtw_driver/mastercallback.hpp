#ifndef MASTERCALLBACK_H
#define MASTERCALLBACK_H

#include <set>

#include "../xsens/xsensdeviceapi.h"
#include "../xsens/xsmutex.h"


typedef std::set<XsDevice*> XsDeviceSet;

class WirelessMasterCallback : public XsCallback
{
public:
	XsDeviceSet getWirelessMTWs() const;

protected:
	virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState);

private:
	mutable XsMutex m_mutex;
	XsDeviceSet m_connectedMTWs;
};

#endif