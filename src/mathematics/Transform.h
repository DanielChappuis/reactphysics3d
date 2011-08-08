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

#ifndef TRANSFORM_H
#define	TRANSFORM_H

// Libraries
#include "Matrix3x3.h"
#include "Vector3D.h"
#include "Quaternion.h"

// ReactPhysiscs3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Transform :
        This class represents a position and an orientation in 3D. It can
        also be seen as representing a translation and a rotation
    -------------------------------------------------------------------
*/
class Transform {
    private :
        Vector3D position;          // Position
        Quaternion orientation;     // Orientation

    public :
        Transform();                                                            // Constructor
        Transform(const Vector3D& position, const Matrix3x3& orientation);      // Constructor
        Transform(const Vector3D& position, const Quaternion& orientation);     // Constructor
        ~Transform();                                                           // Destructor

        const Vector3D& getPosition() const;                                    // Return the origin of the transform
        void setPosition(const Vector3D& position);                             // Set the origin of the transform
        const Quaternion& getOrientation() const;                               // Return the orientation quaternion
        void setOrientation(const Quaternion& orientation);                     // Set the rotation quaternion
        void setToIdentity();                                                   // Set the transform to the identity transform
        void setFromOpenGL(double* openglMatrix);                               // Set the transform from an OpenGL transform matrix
        void getOpenGLMatrix(double* openglMatrix) const;                       // Get the OpenGL matrix of the transform
        Transform inverse() const;                                              // Return the inverse of the transform
        static Transform interpolateTransforms(const Transform& oldTransform,
                                               const Transform& newTransform,
                                               double interpolationFactor);     // Return an interpolated transform

        Vector3D operator*(const Vector3D& vector) const;           // Return the transformed vector
        Transform operator*(const Transform& transform2) const;     // Operator of multiplication of a transform with another one
};

// Return the position of the transform
inline const Vector3D& Transform::getPosition() const {
    return position;
}

// Set the origin of the transform
inline void Transform::setPosition(const Vector3D& position) {
    this->position = position;
}

// Return the rotation matrix
inline const Quaternion& Transform::getOrientation() const {
    return orientation;
}

// Set the rotation matrix of the transform
inline void Transform::setOrientation(const Quaternion& orientation) {
    this->orientation = orientation;
}

// Set the transform to the identity transform
inline void Transform::setToIdentity() {
    position = Vector3D(0.0, 0.0, 0.0);
    orientation = Quaternion::identity();
}

// Set the transform from an OpenGL transform matrix
inline void Transform::setFromOpenGL(double* openglMatrix) {
    Matrix3x3 matrix(openglMatrix[0], openglMatrix[4], openglMatrix[8],
                     openglMatrix[1], openglMatrix[5], openglMatrix[9],
                     openglMatrix[2], openglMatrix[6], openglMatrix[10]);
    orientation = Quaternion(matrix);
    position.setAllValues(openglMatrix[12], openglMatrix[13], openglMatrix[14]);
}

// Get the OpenGL matrix of the transform
inline void Transform::getOpenGLMatrix(double* openglMatrix) const {
    const Matrix3x3& matrix = orientation.getMatrix();
    openglMatrix[0] = matrix.getValue(0, 0); openglMatrix[1] = matrix.getValue(1, 0); openglMatrix[2] = matrix.getValue(2, 0); openglMatrix[3] = 0.0;
    openglMatrix[4] = matrix.getValue(0, 1); openglMatrix[5] = matrix.getValue(1, 1); openglMatrix[6] = matrix.getValue(2, 1); openglMatrix[7] = 0.0;
    openglMatrix[8] = matrix.getValue(0, 2); openglMatrix[9] = matrix.getValue(1, 2); openglMatrix[10] = matrix.getValue(2, 2); openglMatrix[11] = 0.0;
    openglMatrix[12] = position.getX(); openglMatrix[13] = position.getY(); openglMatrix[14] = position.getZ(); openglMatrix[15] = 1.0;
}

// Return the inverse of the transform
inline Transform Transform::inverse() const {
    const Quaternion& invQuaternion = orientation.getInverse();
    Matrix3x3 invMatrix = invQuaternion.getMatrix();
    return Transform(invMatrix * position.getOpposite(), invQuaternion);
}

// Return an interpolated transform
inline Transform Transform::interpolateTransforms(const Transform& oldTransform, const Transform& newTransform, double interpolationFactor) {
    Vector3D interPosition = oldTransform.position * (1.0 - interpolationFactor) + newTransform.position * interpolationFactor;
    Quaternion interOrientation = Quaternion::slerp(oldTransform.orientation, newTransform.orientation, interpolationFactor);
    return Transform(interPosition, interOrientation);
}

// Return the transformed vector
inline Vector3D Transform::operator*(const Vector3D& vector) const {
    return (orientation.getMatrix() * vector) + position;
}

// Operator of multiplication of a transform with another one
inline Transform Transform::operator*(const Transform& transform2) const {
    return Transform(position + orientation.getMatrix() * transform2.position, orientation * transform2.orientation);
}

} // End of the ReactPhysics3D namespace

#endif

