#ifndef FINDCLOSESTUPDATERATE_H
#define FINDCLOSESTUPDATERATE_H

#include <sstream>

#include "../include/xsens/xstypes/xsintarray.h"


int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

#endif