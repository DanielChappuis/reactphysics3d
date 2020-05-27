/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/mathematics/Quaternion.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <cassert>

// Namespace
using namespace reactphysics3d;

// Return a quaternion constructed from Euler angles (in radians)
Quaternion Quaternion::fromEulerAngles(decimal angleX, decimal angleY, decimal angleZ) {

    Quaternion quaternion;
    quaternion.initWithEulerAngles(angleX, angleY, angleZ);

    return quaternion;
}


// Return a quaternion constructed from Euler angles (in radians)
Quaternion Quaternion::fromEulerAngles(const Vector3& eulerAngles) {

    Quaternion quaternion;
    quaternion.initWithEulerAngles(eulerAngles.x, eulerAngles.y, eulerAngles.z);

    return quaternion;
}

// Copy-constructor
Quaternion::Quaternion(const Quaternion& quaternion)
           :x(quaternion.x), y(quaternion.y), z(quaternion.z), w(quaternion.w) {

}

// Create a unit quaternion from a rotation matrix
Quaternion::Quaternion(const Matrix3x3& matrix) {

    // Get the trace of the matrix
    decimal trace = matrix.getTrace();

    decimal r;
    decimal s;

    if (trace < decimal(0.0)) {
        if (matrix[1][1] > matrix[0][0]) {
            if(matrix[2][2] > matrix[1][1]) {
                r = std::sqrt(matrix[2][2] - matrix[0][0] - matrix[1][1] + decimal(1.0));
                s = decimal(0.5) / r;
                
                // Compute the quaternion
                x = (matrix[2][0] + matrix[0][2]) * s;
                y = (matrix[1][2] + matrix[2][1]) * s;
                z = decimal(0.5) * r;
                w = (matrix[1][0] - matrix[0][1]) * s;
            }
            else {
                r = std::sqrt(matrix[1][1] - matrix[2][2] - matrix[0][0] + decimal(1.0));
                s = decimal(0.5) / r;

                // Compute the quaternion
                x = (matrix[0][1] + matrix[1][0]) * s;
                y = decimal(0.5) * r;
                z = (matrix[1][2] + matrix[2][1]) * s;
                w = (matrix[0][2] - matrix[2][0]) * s;
            }
        }
        else if (matrix[2][2] > matrix[0][0]) {
            r = std::sqrt(matrix[2][2] - matrix[0][0] - matrix[1][1] + decimal(1.0));
            s = decimal(0.5) / r;

            // Compute the quaternion
            x = (matrix[2][0] + matrix[0][2]) * s;
            y = (matrix[1][2] + matrix[2][1]) * s;
            z = decimal(0.5) * r;
            w = (matrix[1][0] - matrix[0][1]) * s;
        }
        else {
            r = std::sqrt(matrix[0][0] - matrix[1][1] - matrix[2][2] + decimal(1.0));
            s = decimal(0.5) / r;

            // Compute the quaternion
            x = decimal(0.5) * r;
            y = (matrix[0][1] + matrix[1][0]) * s;
            z = (matrix[2][0] - matrix[0][2]) * s;
            w = (matrix[2][1] - matrix[1][2]) * s;
        }
    }
    else {
        r = std::sqrt(trace + decimal(1.0));
        s = decimal(0.5) / r;

        // Compute the quaternion
        x = (matrix[2][1] - matrix[1][2]) * s;
        y = (matrix[0][2] - matrix[2][0]) * s;
        z = (matrix[1][0] - matrix[0][1]) * s;
        w = decimal(0.5) * r;
    }
}

// Compute the rotation angle (in radians) and the rotation axis
/// This method is used to get the rotation angle (in radian) and the unit
/// rotation axis of an orientation quaternion.
void Quaternion::getRotationAngleAxis(decimal& angle, Vector3& axis) const {

    // Compute the roation angle
    angle = std::acos(w) * decimal(2.0);

    // Compute the 3D rotation axis
    Vector3 rotationAxis(x, y, z);

    // Normalize the rotation axis
    rotationAxis = rotationAxis.getUnit();

    // Set the rotation axis values
    axis.setAllValues(rotationAxis.x, rotationAxis.y, rotationAxis.z);
}

// Return the orientation matrix corresponding to this quaternion
Matrix3x3 Quaternion::getMatrix() const {

    decimal nQ = x*x + y*y + z*z + w*w;
    decimal s = 0.0;

    if (nQ > decimal(0.0)) {
        s = decimal(2.0) / nQ;
    }

    // Computations used for optimization (less multiplications)
    decimal xs = x*s;
    decimal ys = y*s;
    decimal zs = z*s;
    decimal wxs = w*xs;
    decimal wys = w*ys;
    decimal wzs = w*zs;
    decimal xxs = x*xs;
    decimal xys = x*ys;
    decimal xzs = x*zs;
    decimal yys = y*ys;
    decimal yzs = y*zs;
    decimal zzs = z*zs;

    // Create the matrix corresponding to the quaternion
    return Matrix3x3(decimal(1.0) - yys - zzs, xys-wzs, xzs + wys,
                     xys + wzs, decimal(1.0) - xxs - zzs, yzs-wxs,
                     xzs-wys, yzs + wxs, decimal(1.0) - xxs - yys);
}

// Compute the spherical linear interpolation between two quaternions.
/// The t argument has to be such that 0 <= t <= 1. This method is static.
Quaternion Quaternion::slerp(const Quaternion& quaternion1,
                             const Quaternion& quaternion2, decimal t) {
    assert(t >= decimal(0.0) && t <= decimal(1.0));

    decimal invert = 1.0;

    // Compute cos(theta) using the quaternion scalar product
    decimal cosineTheta = quaternion1.dot(quaternion2);

    // Take care of the sign of cosineTheta
    if (cosineTheta < decimal(0.0)) {
			cosineTheta = -cosineTheta;
			invert = -1.0;
    }

    // Because of precision, if cos(theta) is nearly 1,
    // therefore theta is nearly 0 and we can write
    // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
    const decimal epsilon = decimal(0.00001);
    if(1-cosineTheta < epsilon) {
        return quaternion1 * (decimal(1.0)-t) + quaternion2 * (t * invert);
    }

    // Compute the theta angle
    decimal theta = std::acos(cosineTheta);

    // Compute sin(theta)
    decimal sineTheta = std::sin(theta);

    // Compute the two coefficients that are in the spherical linear interpolation formula
    decimal coeff1 = std::sin((decimal(1.0)-t)*theta) / sineTheta;
    decimal coeff2 = std::sin(t*theta) / sineTheta * invert;

    // Compute and return the interpolated quaternion
    return quaternion1 * coeff1 + quaternion2 * coeff2;
}

// Initialize the quaternion using Euler angles
void Quaternion::initWithEulerAngles(decimal angleX, decimal angleY, decimal angleZ) {

    decimal angle = angleX * decimal(0.5);
    const decimal sinX = std::sin(angle);
    const decimal cosX = std::cos(angle);

    angle = angleY * decimal(0.5);
    const decimal sinY = std::sin(angle);
    const decimal cosY = std::cos(angle);

    angle = angleZ * decimal(0.5);
    const decimal sinZ = std::sin(angle);
    const decimal cosZ = std::cos(angle);

    const decimal cosYcosZ = cosY * cosZ;
    const decimal sinYcosZ = sinY * cosZ;
    const decimal cosYsinZ = cosY * sinZ;
    const decimal sinYsinZ = sinY * sinZ;

    x = sinX * cosYcosZ - cosX * sinYsinZ;
    y = cosX * sinYcosZ + sinX * cosYsinZ;
    z = cosX * cosYsinZ - sinX * sinYcosZ;
    w = cosX * cosYcosZ + sinX * sinYsinZ;

    // Normalize the quaternion
    normalize();
}
