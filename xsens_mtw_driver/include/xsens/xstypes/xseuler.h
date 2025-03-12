
//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSEULER_H
#define XSEULER_H

#include "xsmath.h"
#include <math.h>

#ifdef __cplusplus
#include "xsvector.h"
extern "C" {
#endif
#ifndef __cplusplus
#define XSEULER_INITIALIZER { { { XsMath_zero,XsMath_zero,XsMath_zero } } }
struct XsVector;
#endif

struct XsEuler;
struct XsQuaternion;
struct XsMatrix;

XSTYPES_DLL_API void XsEuler_destruct(struct XsEuler* thisPtr);
XSTYPES_DLL_API int XsEuler_empty(const struct XsEuler* thisPtr);
XSTYPES_DLL_API void XsEuler_fromQuaternion(struct XsEuler* thisPtr, const struct XsQuaternion* quat);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsEuler
{
#ifdef __cplusplus
	//! \brief Constructor that creates an Euler object with all angles 0
	inline XsEuler() : m_union({{XsMath_zero, XsMath_zero, XsMath_zero}}) {}
	//! \brief Constructor that creates an Euler object the specified angles
	inline XsEuler(XsReal x_, XsReal y_, XsReal z_) : m_union({{x_, y_, z_}}) {}
	//! \brief Constructor that creates an Euler object from \a other
    inline XsEuler(const XsEuler& other) : m_union(other.m_union) {}
	//! \brief Constructor that creates an Euler object from \a other
	inline explicit XsEuler(const XsVector& other) : m_union({{other[0], other[1], other[2]}}) {}

	//! \brief \copybrief XsEuler_fromQuaternion
	inline explicit XsEuler(const XsQuaternion& q)
	{
		XsEuler_fromQuaternion(this, &q);
	}

	//! \brief Assigns the \a other XsEuler object to this one
	inline XsEuler& operator=(const XsEuler& other)
	{
		m_union.m_cartesian.m_x = other.m_union.m_cartesian.m_x;
		m_union.m_cartesian.m_y = other.m_union.m_cartesian.m_y;
		m_union.m_cartesian.m_z = other.m_union.m_cartesian.m_z;
		return *this;
	}

	//! \brief Returns the \a index'th euler angle in the object
	inline XsReal operator[](XsSize index) const
	{
		assert(index <= 2);
		return m_union.m_data[index];
	}

	//! \brief Returns a reference to the \a index'th euler angle in the object
	inline XsReal& operator[](XsSize index)
	{
		assert(index <= 2);
		return m_union.m_data[index];
	}

	//! \brief Returns true if all angles in this object are zero
	inline bool empty() const
	{
		return m_union.m_cartesian.m_x == XsMath_zero &&
			   m_union.m_cartesian.m_y == XsMath_zero &&
			   m_union.m_cartesian.m_z == XsMath_zero;
	}

	//! \brief Return a const pointer to the internal data
	inline const XsReal* data() const
	{
		return m_union.m_data;
	}

	//! \brief \copybrief XsEuler_fromQuaternion
	inline XsEuler& fromQuaternion(const XsQuaternion& quat)
	{
		XsEuler_fromQuaternion(this, &quat);
		return *this;
	}

	/*! \brief Returns true if the values in \a other are exactly equal to this
	*/
	inline bool operator == (const XsEuler& other) const
	{
		return m_union.m_euler.m_roll  == other.m_union.m_euler.m_roll  &&
			   m_union.m_euler.m_pitch == other.m_union.m_euler.m_pitch &&
			   m_union.m_euler.m_yaw   == other.m_union.m_euler.m_yaw;
	}

	/*! \brief Returns true if the values in \a other are different from this
	*/
	inline bool operator != (const XsEuler& other) const
	{
		return m_union.m_euler.m_roll  != other.m_union.m_euler.m_roll  ||
			   m_union.m_euler.m_pitch != other.m_union.m_euler.m_pitch ||
			   m_union.m_euler.m_yaw   != other.m_union.m_euler.m_yaw;
	}

	//! \brief Returns the roll or x value
	inline XsReal roll() const
	{
		return m_union.m_euler.m_roll;
	}
	//! \brief Returns the pitch or y value
	inline XsReal pitch() const
	{
		return m_union.m_euler.m_pitch;
	}
	//! \brief Returns the yaw or z value
	inline XsReal yaw() const
	{
		return m_union.m_euler.m_yaw;
	}

	//! \brief Returns the x or roll value
	inline XsReal x() const
	{
		return m_union.m_cartesian.m_x;
	}
	//! \brief Returns the y or pitch value
	inline XsReal y() const
	{
		return m_union.m_cartesian.m_y;
	}
	//! \brief Returns the z or yaw value
	inline XsReal z() const
	{
		return m_union.m_cartesian.m_z;
	}

	//! \brief Returns true if the values of this and \a other are within \a tolerance of each other
	inline bool isEqual(const XsEuler& other, XsReal tolerance) const
	{
		return	fabs(m_union.m_cartesian.m_x - other.m_union.m_cartesian.m_x) <= tolerance &&
				fabs(m_union.m_cartesian.m_y - other.m_union.m_cartesian.m_y) <= tolerance &&
				fabs(m_union.m_cartesian.m_z - other.m_union.m_cartesian.m_z) <= tolerance;
	}

private:
#endif

	union EulerUnion
	{
		struct CartesianComponents
		{
			XsReal m_x;		//!< Stores the x component of the euler triplet
			XsReal m_y;		//!< Stores the y component of the euler triplet
			XsReal m_z;		//!< Stores the z component of the euler triplet
		} m_cartesian;
		struct EulerComponents
		{
			XsReal m_roll;		//!< Stores the roll component of the euler triplet
			XsReal m_pitch;		//!< Stores the pitch component of the euler triplet
			XsReal m_yaw;		//!< Stores the yaw component of the euler triplet
		} m_euler;	
		XsReal m_data[3];	//!< Stores the euler triplet in an array of three elements
	} m_union;
};

typedef struct XsEuler XsEuler;

#endif
