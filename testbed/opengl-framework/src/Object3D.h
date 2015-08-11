/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2013 Daniel Chappuis                                            *
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

#ifndef OBJECT3D_H
#define OBJECT3D_H

// Libraries
#include "maths/Vector3.h"
#include "maths/Matrix4.h"

namespace openglframework {

// Class Object3D
// This class represent a generic 3D object on the scene.
class Object3D {

    protected:

        // -------------------- Attributes -------------------- //

        // Transformation matrix that convert local-space
        // coordinates to world-space coordinates
        Matrix4 mTransformMatrix;

    public:

        // -------------------- Methods -------------------- //

        // Constructor
        Object3D();

        // Destructor
        virtual ~Object3D();

        // Return the transform matrix
        const Matrix4& getTransformMatrix() const;

        // Set the transform matrix
        void setTransformMatrix(const Matrix4& matrix);

        // Set to the identity transform
        void setToIdentity();

        // Return the origin of object in world-space
        Vector3 getOrigin() const;

        // Translate the object in world-space
        void translateWorld(const Vector3& v);

        // Translate the object in local-space
        void translateLocal(const Vector3& v);

        // Rotate the object in world-space
        void rotateWorld(const Vector3& axis, float angle);

        // Rotate the object in local-space
        void rotateLocal(const Vector3& axis, float angle);

        // Rotate around a world-space point
        void rotateAroundWorldPoint(const Vector3& axis, float angle, const Vector3& point);

        // Rotate around a local-space point
        void rotateAroundLocalPoint(const Vector3& axis, float angle, const Vector3& worldPoint);
};

// Return the transform matrix
inline const Matrix4& Object3D::getTransformMatrix() const {
    return mTransformMatrix;
}

// Set the transform matrix
inline void Object3D::setTransformMatrix(const Matrix4& matrix) {
    mTransformMatrix = matrix;
}

// Set to the identity transform
inline void Object3D::setToIdentity() {
    mTransformMatrix.setToIdentity();
}

 // Return the origin of object in world-space
inline Vector3 Object3D::getOrigin() const {
    return mTransformMatrix * Vector3(0.0, 0.0, 0.0);
}

// Translate the object in world-space
inline void Object3D::translateWorld(const Vector3& v) {
    mTransformMatrix = Matrix4::translationMatrix(v) * mTransformMatrix;
}

// Translate the object in local-space
inline void Object3D::translateLocal(const Vector3& v) {
    mTransformMatrix = mTransformMatrix * Matrix4::translationMatrix(v);
}

// Rotate the object in world-space
inline void Object3D::rotateWorld(const Vector3& axis, float angle) {
    mTransformMatrix = Matrix4::rotationMatrix(axis, angle) * mTransformMatrix;
}

// Rotate the object in local-space
inline void Object3D::rotateLocal(const Vector3& axis, float angle) {
    mTransformMatrix = mTransformMatrix * Matrix4::rotationMatrix(axis, angle);
}

// Rotate the object around a world-space point
inline void Object3D::rotateAroundWorldPoint(const Vector3& axis, float angle,
                                             const Vector3& worldPoint) {
    mTransformMatrix = Matrix4::translationMatrix(worldPoint) * Matrix4::rotationMatrix(axis, angle)
                       * Matrix4::translationMatrix(-worldPoint) * mTransformMatrix;
}

// Rotate the object around a local-space point
inline void Object3D::rotateAroundLocalPoint(const Vector3& axis, float angle,
                                             const Vector3& worldPoint) {

    // Convert the world point into the local coordinate system
    Vector3 localPoint = mTransformMatrix.getInverse() * worldPoint;

    mTransformMatrix = mTransformMatrix * Matrix4::translationMatrix(localPoint)
                       * Matrix4::rotationMatrix(axis, angle)
                       * Matrix4::translationMatrix(-localPoint);
}

}

#endif
