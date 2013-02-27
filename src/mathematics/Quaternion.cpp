/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

// Libraries
#include "Quaternion.h"
#include "Vector3.h"
#include <cassert>

// Namespace
using namespace reactphysics3d;

// Constructor of the class
Quaternion::Quaternion()
           :mX(0.0), mY(0.0), mZ(0.0), mW(0.0) {

}

// Constructor with arguments
Quaternion::Quaternion(decimal x, decimal y, decimal z, decimal w)
           :mX(x), mY(y), mZ(z), mW(w) {

}

// Constructor with the component w and the vector v=(x y z)
Quaternion::Quaternion(decimal w, const Vector3& v)
           :mX(v.x), mY(v.y), mZ(v.z), mW(w) {

}

// Copy-constructor
Quaternion::Quaternion(const Quaternion& quaternion)
           :mX(quaternion.mX), mY(quaternion.mY), mZ(quaternion.mZ), mW(quaternion.mW) {

}

// Create a unit quaternion from a rotation matrix
Quaternion::Quaternion(const Matrix3x3& matrix) {

    // Get the trace of the matrix
    decimal trace = matrix.getTrace();

    decimal array[3][3];
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            array[i][j] = matrix.getValue(i, j);
        }
    }

    decimal r;
    decimal s;

    if (trace < 0.0) {
        if (array[1][1] > array[0][0]) {
            if(array[2][2] > array[1][1]) {
                r = sqrt(array[2][2] - array[0][0] - array[1][1] + 1.0);
                s = 0.5 / r;
                
                // Compute the quaternion
                mX = (array[2][0] + array[0][2])*s;
                mY = (array[1][2] + array[2][1])*s;
                mZ = 0.5*r;
                mW = (array[1][0] - array[0][1])*s;
            }
            else {
                r = sqrt(array[1][1] - array[2][2] - array[0][0] + 1.0);
                s = 0.5 / r;

                // Compute the quaternion
                mX = (array[0][1] + array[1][0])*s;
                mY = 0.5 * r;
                mZ = (array[1][2] + array[2][1])*s;
                mW = (array[0][2] - array[2][0])*s;
            }
        }
        else if (array[2][2] > array[0][0]) {
            r = sqrt(array[2][2] - array[0][0] - array[1][1] + 1.0);
            s = 0.5 / r;

            // Compute the quaternion
            mX = (array[2][0] + array[0][2])*s;
            mY = (array[1][2] + array[2][1])*s;
            mZ = 0.5 * r;
            mW = (array[1][0] - array[0][1])*s;
        }
        else {
            r = sqrt(array[0][0] - array[1][1] - array[2][2] + 1.0);
            s = 0.5 / r;

            // Compute the quaternion
            mX = 0.5 * r;
            mY = (array[0][1] + array[1][0])*s;
            mZ = (array[2][0] - array[0][2])*s;
            mW = (array[2][1] - array[1][2])*s;
        }
    }
    else {
        r = sqrt(trace + 1.0);
        s = 0.5/r;

        // Compute the quaternion
        mX = (array[2][1]-array[1][2])*s;
        mY = (array[0][2]-array[2][0])*s;
        mZ = (array[1][0]-array[0][1])*s;
        mW = 0.5 * r;
    }
}

// Destructor
Quaternion::~Quaternion() {

}

// Compute the rotation angle (in radians) and the 3D rotation axis
// This method is used to get the rotation angle (in radian) and the unit
// rotation axis of an orientation quaternion.
void Quaternion::getRotationAngleAxis(decimal& angle, Vector3& axis) const {
    Quaternion quaternion;

    // If the quaternion is unit
    if (length() == 1.0) {
        quaternion = *this;
    }
    else {
        // We compute the unit quaternion
        quaternion = getUnit();
    }

    // Compute the roation angle
    angle = acos(quaternion.mW) * 2.0;

    // Compute the 3D rotation axis
    Vector3 rotationAxis(quaternion.mX, quaternion.mY, quaternion.mZ);

    // Normalize the rotation axis
    rotationAxis = rotationAxis.getUnit();

    // Set the rotation axis values
    axis.setAllValues(rotationAxis.x, rotationAxis.y, rotationAxis.z);
}

// Return the orientation matrix corresponding to this quaternion
Matrix3x3 Quaternion::getMatrix() const {

    decimal nQ = mX*mX + mY*mY + mZ*mZ + mW*mW;
    decimal s = 0.0;

    if (nQ > 0.0) {
        s = 2.0/nQ;
    }

    // Computations used for optimization (less multiplications)
    decimal xs = mX*s;
    decimal ys = mY*s;
    decimal zs = mZ*s;
    decimal wxs = mW*xs;
    decimal wys = mW*ys;
    decimal wzs = mW*zs;
    decimal xxs = mX*xs;
    decimal xys = mX*ys;
    decimal xzs = mX*zs;
    decimal yys = mY*ys;
    decimal yzs = mY*zs;
    decimal zzs = mZ*zs;

    // Create the matrix corresponding to the quaternion
    return Matrix3x3(1.0-yys-zzs, xys-wzs, xzs + wys,
                     xys + wzs, 1.0-xxs-zzs, yzs-wxs,
                     xzs-wys, yzs + wxs, 1.0-xxs-yys);
}

// Compute the spherical linear interpolation between two quaternions.
// The t argument has to be such that 0 <= t <= 1. This method is static.
Quaternion Quaternion::slerp(const Quaternion& quaternion1,
                             const Quaternion& quaternion2, decimal t) {
    assert(t >= 0.0 && t <= 1.0);

    decimal invert = 1.0;

    // Compute cos(theta) using the quaternion scalar product
    decimal cosineTheta = quaternion1.dot(quaternion2);

    // Take care of the sign of cosineTheta
    if (cosineTheta < 0.0) {
			cosineTheta = -cosineTheta;
			invert = -1.0;
    }

    // Because of precision, if cos(theta) is nearly 1,
    // therefore theta is nearly 0 and we can write
    // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
    const decimal epsilon = 0.00001;
    if(1-cosineTheta < epsilon) {
        return quaternion1 * (1.0-t) + quaternion2 * (t * invert);
    }

    // Compute the theta angle
    decimal theta = acos(cosineTheta);

    // Compute sin(theta)
    decimal sineTheta = sin(theta);

    // Compute the two coefficients that are in the spherical linear interpolation formula
    decimal coeff1 = sin((1.0-t)*theta) / sineTheta;
    decimal coeff2 = sin(t*theta) / sineTheta * invert;

    // Compute and return the interpolated quaternion
    return quaternion1 * coeff1 + quaternion2 * coeff2;
}


