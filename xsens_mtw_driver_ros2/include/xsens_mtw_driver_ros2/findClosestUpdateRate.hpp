#ifndef FINDCLOSESTUPDATERATE_H
#define FINDCLOSESTUPDATERATE_H


#include "../include/xsens/xstypes/xsintarray.h"


int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

#endif