/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

// Libraries
#include "Quaternion.h"
#include "Vector3.h"
#include <cassert>

// Namespace
using namespace reactphysics3d;

// Constructor of the class
Quaternion::Quaternion()
           :x(0.0), y(0.0), z(0.0), w(0.0) {

}

// Constructor with arguments
Quaternion::Quaternion(double x, double y, double z, double w)
           :x(x), y(y), z(z), w(w) {

}

// Constructor with the component w and the vector v=(x y z)
Quaternion::Quaternion(double w, const Vector3& v)
           :x(v.getX()), y(v.getY()), z(v.getZ()), w(w) {

}

// Copy-constructor
Quaternion::Quaternion(const Quaternion& quaternion)
           :x(quaternion.x), y(quaternion.y), z(quaternion.z), w(quaternion.w) {

}

// Create a unit quaternion from a rotation matrix
Quaternion::Quaternion(const Matrix3x3& matrix) {

    // Get the trace of the matrix
    double trace = matrix.getTrace();

    double array[3][3];
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            array[i][j] = matrix.getValue(i, j);
        }
    }

    double r;
    double s;

    if (trace < 0.0) {
        if (array[1][1] > array[0][0]) {
            if(array[2][2] > array[1][1]) {
                r = sqrt(array[2][2] - array[0][0] - array[1][1] + 1.0);
                s = 0.5 / r;
                
                // Compute the quaternion
                x = (array[2][0] + array[0][2])*s;
                y = (array[1][2] + array[2][1])*s;
                z = 0.5*r;
                w = (array[1][0] - array[0][1])*s;
            }
            else {
                r = sqrt(array[1][1] - array[2][2] - array[0][0] + 1.0);
                s = 0.5 / r;

                // Compute the quaternion
                x = (array[0][1] + array[1][0])*s;
                y = 0.5 * r;
                z = (array[1][2] + array[2][1])*s;
                w = (array[0][2] - array[2][0])*s;
            }
        }
        else if (array[2][2] > array[0][0]) {
            r = sqrt(array[2][2] - array[0][0] - array[1][1] + 1.0);
            s = 0.5 / r;

            // Compute the quaternion
            x = (array[2][0] + array[0][2])*s;
            y = (array[1][2] + array[2][1])*s;
            z = 0.5 * r;
            w = (array[1][0] - array[0][1])*s;
        }
        else {
            r = sqrt(array[0][0] - array[1][1] - array[2][2] + 1.0);
            s = 0.5 / r;

            // Compute the quaternion
            x = 0.5 * r;
            y = (array[0][1] + array[1][0])*s;
            z = (array[2][0] - array[0][2])*s;
            w = (array[2][1] - array[1][2])*s;
        }
    }
    else {
        r = sqrt(trace + 1.0);
        s = 0.5/r;

        // Compute the quaternion
        x = (array[2][1]-array[1][2])*s;
        y = (array[0][2]-array[2][0])*s;
        z = (array[1][0]-array[0][1])*s;
        w = 0.5 * r;
    }
}

// Destructor
Quaternion::~Quaternion() {

}

// Compute the rotation angle (in radians) and the 3D rotation axis
// This method is used to get the rotation angle (in radian) and the unit
// rotation axis of an orientation quaternion.
void Quaternion::getRotationAngleAxis(double& angle, Vector3& axis) const {
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
    angle = acos(quaternion.w) * 2.0;

    // Compute the 3D rotation axis
    Vector3 rotationAxis(quaternion.x, quaternion.y, quaternion.z);

    // Normalize the rotation axis
    rotationAxis = rotationAxis.getUnit();

    // Set the rotation axis values
    axis.setAllValues(rotationAxis.getX(), rotationAxis.getY(), rotationAxis.getZ());
}

// Return the orientation matrix corresponding to this quaternion
Matrix3x3 Quaternion::getMatrix() const {

    double nQ = x*x + y*y + z*z + w*w;
    double s = 0.0;

    if (nQ > 0.0) {
        s = 2.0/nQ;
    }

    // Computations used for optimization (less multiplications)
    double xs = x*s;
    double ys = y*s;
    double zs = z*s;
    double wxs = w*xs;
    double wys = w*ys;
    double wzs = w*zs;
    double xxs = x*xs;
    double xys = x*ys;
    double xzs = x*zs;
    double yys = y*ys;
    double yzs = y*zs;
    double zzs = z*zs;

    // Create the matrix corresponding to the quaternion
    return Matrix3x3(1.0-yys-zzs, xys-wzs, xzs + wys,
                     xys + wzs, 1.0-xxs-zzs, yzs-wxs,
                     xzs-wys, yzs + wxs, 1.0-xxs-yys);
}

// Compute the spherical linear interpolation between two quaternions.
// The t argument has to be such that 0 <= t <= 1. This method is static.
Quaternion Quaternion::slerp(const Quaternion& quaternion1, const Quaternion& quaternion2, double t) {
    assert(t >= 0.0 && t <= 1.0);

    double invert = 1.0;

    // Compute cos(theta) using the quaternion scalar product
    double cosineTheta = quaternion1.dot(quaternion2);

    // Take care of the sign of cosineTheta
    if (cosineTheta < 0.0) {
			cosineTheta = -cosineTheta;
			invert = -1.0;
    }

    // Because of precision, if cos(theta) is nearly 1, therefore theta is nearly 0 and we can write
    // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
    if(1-cosineTheta < EPSILON_TEST) {
        return quaternion1 * (1.0-t) + quaternion2 * (t * invert);
    }

    // Compute the theta angle
    double theta = acos(cosineTheta);

    // Compute sin(theta)
    double sineTheta = sin(theta);

    // Compute the two coefficients that are in the spherical linear interpolation formula
    double coeff1 = sin((1.0-t)*theta) / sineTheta;
    double coeff2 = sin(t*theta) / sineTheta * invert;

    // Compute and return the interpolated quaternion
    return quaternion1 * coeff1 + quaternion2 * coeff2;
}


