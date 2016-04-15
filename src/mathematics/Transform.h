/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_TRANSFORM_H
#define	REACTPHYSICS3D_TRANSFORM_H

// Libraries
#include "Matrix3x3.h"
#include "Vector3.h"
#include "Quaternion.h"

// ReactPhysiscs3D namespace
namespace reactphysics3d {

// Class Transform
/**
 * This class represents a position and an orientation in 3D. It can
 * also be seen as representing a translation and a rotation.
 */
class Transform {

    private :

        // -------------------- Attributes -------------------- //

        /// Position
        Vector3 mPosition;

        /// Orientation
        Quaternion mOrientation;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Transform();

        /// Constructor
        Transform(const Vector3& position, const Matrix3x3& orientation);

        /// Constructor
        Transform(const Vector3& position, const Quaternion& orientation);

        /// Destructor
        ~Transform();

        /// Copy-constructor
        Transform(const Transform& transform);

        /// Return the origin of the transform
        const Vector3& getPosition() const;

        /// Set the origin of the transform
        void setPosition(const Vector3& position);

        /// Return the orientation quaternion
        const Quaternion& getOrientation() const;

        /// Set the rotation quaternion
        void setOrientation(const Quaternion& orientation);

        /// Set the transform to the identity transform
        void setToIdentity();

        /// Set the transform from an OpenGL transform matrix
        void setFromOpenGL(decimal* openglMatrix);

        /// Get the OpenGL matrix of the transform
        void getOpenGLMatrix(decimal* openglMatrix) const;

        /// Return the inverse of the transform
        Transform getInverse() const;

        /// Return an interpolated transform
        static Transform interpolateTransforms(const Transform& oldTransform,
                                               const Transform& newTransform,
                                               decimal interpolationFactor);

        /// Return the identity transform
        static Transform identity();

        /// Return the transformed vector
        Vector3 operator*(const Vector3& vector) const;

        /// Operator of multiplication of a transform with another one
        Transform operator*(const Transform& transform2) const;

        /// Return true if the two transforms are equal
        bool operator==(const Transform& transform2) const;

        /// Return true if the two transforms are different
        bool operator!=(const Transform& transform2) const;

        /// Assignment operator
        Transform& operator=(const Transform& transform);
};

// Return the position of the transform
inline const Vector3& Transform::getPosition() const {
    return mPosition;
}

// Set the origin of the transform
inline void Transform::setPosition(const Vector3& position) {
    mPosition = position;
}

// Return the rotation matrix
inline const Quaternion& Transform::getOrientation() const {
    return mOrientation;
}

// Set the rotation matrix of the transform
inline void Transform::setOrientation(const Quaternion& orientation) {
    mOrientation = orientation;
}

// Set the transform to the identity transform
inline void Transform::setToIdentity() {
    mPosition = Vector3(0.0, 0.0, 0.0);
    mOrientation = Quaternion::identity();
}                                           

// Set the transform from an OpenGL transform matrix
inline void Transform::setFromOpenGL(decimal* openglMatrix) {
    Matrix3x3 matrix(openglMatrix[0], openglMatrix[4], openglMatrix[8],
                     openglMatrix[1], openglMatrix[5], openglMatrix[9],
                     openglMatrix[2], openglMatrix[6], openglMatrix[10]);
    mOrientation = Quaternion(matrix);
    mPosition.setAllValues(openglMatrix[12], openglMatrix[13], openglMatrix[14]);
}

// Get the OpenGL matrix of the transform
inline void Transform::getOpenGLMatrix(decimal* openglMatrix) const {
    const Matrix3x3& matrix = mOrientation.getMatrix();
    openglMatrix[0] = matrix[0][0]; openglMatrix[1] = matrix[1][0];
    openglMatrix[2] = matrix[2][0]; openglMatrix[3] = 0.0;
    openglMatrix[4] = matrix[0][1]; openglMatrix[5] = matrix[1][1];
    openglMatrix[6] = matrix[2][1]; openglMatrix[7] = 0.0;
    openglMatrix[8] = matrix[0][2]; openglMatrix[9] = matrix[1][2];
    openglMatrix[10] = matrix[2][2]; openglMatrix[11] = 0.0;
    openglMatrix[12] = mPosition.x; openglMatrix[13] = mPosition.y;
    openglMatrix[14] = mPosition.z; openglMatrix[15] = 1.0;
}

// Return the inverse of the transform
inline Transform Transform::getInverse() const {
    const Quaternion& invQuaternion = mOrientation.getInverse();
    Matrix3x3 invMatrix = invQuaternion.getMatrix();
    return Transform(invMatrix * (-mPosition), invQuaternion);
}

// Return an interpolated transform
inline Transform Transform::interpolateTransforms(const Transform& oldTransform,
                                                  const Transform& newTransform,
                                                  decimal interpolationFactor) {

    Vector3 interPosition = oldTransform.mPosition * (decimal(1.0) - interpolationFactor) +
                            newTransform.mPosition * interpolationFactor;

    Quaternion interOrientation = Quaternion::slerp(oldTransform.mOrientation,
                                                    newTransform.mOrientation,
                                                    interpolationFactor);

    return Transform(interPosition, interOrientation);
}

// Return the identity transform
inline Transform Transform::identity() {
    return Transform(Vector3(0, 0, 0), Quaternion::identity());
}

// Return the transformed vector
inline Vector3 Transform::operator*(const Vector3& vector) const {
    return (mOrientation.getMatrix() * vector) + mPosition;
}

// Operator of multiplication of a transform with another one
inline Transform Transform::operator*(const Transform& transform2) const {
    return Transform(mPosition + mOrientation.getMatrix() * transform2.mPosition,
                     mOrientation * transform2.mOrientation);
}

// Return true if the two transforms are equal
inline bool Transform::operator==(const Transform& transform2) const {
    return (mPosition == transform2.mPosition) && (mOrientation == transform2.mOrientation);
}    

// Return true if the two transforms are different
inline bool Transform::operator!=(const Transform& transform2) const {
    return !(*this == transform2);
}

// Assignment operator
inline Transform& Transform::operator=(const Transform& transform) {
    if (&transform != this) {
        mPosition = transform.mPosition;
        mOrientation = transform.mOrientation;
    }
    return *this;
}

}

#endif

