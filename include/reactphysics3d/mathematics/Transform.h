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

#ifndef REACTPHYSICS3D_TRANSFORM_H
#define	REACTPHYSICS3D_TRANSFORM_H

// Libraries
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/mathematics/Quaternion.h>

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
        ~Transform() = default;

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

        /// Return true if it is a valid transform
        bool isValid() const;

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

        /// Return the string representation
        std::string to_string() const;

};

// Constructor
inline Transform::Transform() : mPosition(Vector3(0.0, 0.0, 0.0)), mOrientation(Quaternion::identity()) {

}

// Constructor
inline Transform::Transform(const Vector3& position, const Matrix3x3& orientation)
          : mPosition(position), mOrientation(Quaternion(orientation)) {

}

// Constructor
inline Transform::Transform(const Vector3& position, const Quaternion& orientation)
          : mPosition(position), mOrientation(orientation) {

}

// Copy-constructor
inline Transform::Transform(const Transform& transform)
          : mPosition(transform.mPosition), mOrientation(transform.mOrientation) {

}

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

// Return the inverse of the transform
inline Transform Transform::getInverse() const {
    const Quaternion& invQuaternion = mOrientation.getInverse();
    return Transform(invQuaternion * (-mPosition), invQuaternion);
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

// Return true if it is a valid transform
inline bool Transform::isValid() const {
    return mPosition.isFinite() && mOrientation.isValid();
}

// Return the transformed vector
inline Vector3 Transform::operator*(const Vector3& vector) const {
    return (mOrientation * vector) + mPosition;
}

// Operator of multiplication of a transform with another one
inline Transform Transform::operator*(const Transform& transform2) const {

    // The following code is equivalent to this
    //return Transform(mPosition + mOrientation * transform2.mPosition,
    //                 mOrientation * transform2.mOrientation);

    const decimal prodX = mOrientation.w * transform2.mPosition.x + mOrientation.y * transform2.mPosition.z
                          - mOrientation.z * transform2.mPosition.y;
    const decimal prodY = mOrientation.w * transform2.mPosition.y + mOrientation.z * transform2.mPosition.x
                          - mOrientation.x * transform2.mPosition.z;
    const decimal prodZ = mOrientation.w * transform2.mPosition.z + mOrientation.x * transform2.mPosition.y
                          - mOrientation.y * transform2.mPosition.x;
    const decimal prodW = -mOrientation.x * transform2.mPosition.x - mOrientation.y * transform2.mPosition.y
                          - mOrientation.z * transform2.mPosition.z;

    return Transform(Vector3(mPosition.x + mOrientation.w * prodX - prodY * mOrientation.z + prodZ * mOrientation.y - prodW * mOrientation.x,
                             mPosition.y + mOrientation.w * prodY - prodZ * mOrientation.x + prodX * mOrientation.z - prodW * mOrientation.y,
                             mPosition.z + mOrientation.w * prodZ - prodX * mOrientation.y + prodY * mOrientation.x - prodW * mOrientation.z),
                     Quaternion(mOrientation.w * transform2.mOrientation.x + transform2.mOrientation.w * mOrientation.x
                       + mOrientation.y * transform2.mOrientation.z - mOrientation.z * transform2.mOrientation.y,
                      mOrientation.w * transform2.mOrientation.y + transform2.mOrientation.w * mOrientation.y
                       + mOrientation.z * transform2.mOrientation.x - mOrientation.x * transform2.mOrientation.z,
                      mOrientation.w * transform2.mOrientation.z + transform2.mOrientation.w * mOrientation.z
                       + mOrientation.x * transform2.mOrientation.y - mOrientation.y * transform2.mOrientation.x,
                      mOrientation.w * transform2.mOrientation.w - mOrientation.x * transform2.mOrientation.x
                       - mOrientation.y * transform2.mOrientation.y - mOrientation.z * transform2.mOrientation.z));
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

// Get the string representation
inline std::string Transform::to_string() const {
    return "Transform(" + mPosition.to_string() + "," + mOrientation.to_string() + ")";
}

}

#endif

