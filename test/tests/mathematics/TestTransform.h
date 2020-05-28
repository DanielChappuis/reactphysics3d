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

#ifndef TEST_TRANSFORM_H
#define TEST_TRANSFORM_H

// Libraries
#include "Test.h"
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestTransform
/**
 * Unit test for the Transform class
 */
class TestTransform : public Test {

    private :

        // ---------- Atributes ---------- //

        /// Identity transform
        Transform mIdentityTransform;

        /// First example transform
        Transform mTransform1;

        /// Second example transform
        Transform mTransform2;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestTransform(const std::string& name) : Test(name) {

            mIdentityTransform.setToIdentity();

            Vector3 unitVec(1, 1, 1);
            unitVec.normalize();

            decimal sinA = std::sin(PI/8.0f);
            decimal cosA = std::cos(PI/8.0f);
            mTransform1 = Transform(Vector3(4, 5, 6), Quaternion(sinA * unitVec, cosA));

            decimal sinB = std::sin(PI/3.0f);
            decimal cosB = std::cos(PI/3.0f);
            mTransform2 = Transform(Vector3(8, 45, -6), Quaternion(sinB * unitVec, cosB));
        }

        /// Run the tests
        void run() {
            testConstructors();
            testGetSet();
            testInverse();
            testGetSetOpenGLMatrix();
            testInterpolateTransform();
            testIdentity();
            testOperators();
        }

        /// Test the constructors
        void testConstructors() {
            Transform transform1(Vector3(1, 2, 3), Quaternion(6, 7, 8, 9));
            Transform transform2(Vector3(4, 5, 6), Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1));
            Transform transform3(transform1);
            rp3d_test(transform1.getPosition() == Vector3(1, 2, 3));
            rp3d_test(transform1.getOrientation() == Quaternion(6, 7, 8, 9));
            rp3d_test(transform2.getPosition() == Vector3(4, 5, 6));
            rp3d_test(transform2.getOrientation() == Quaternion::identity());
            rp3d_test(transform3 == transform1);
        }

        /// Test getter and setter
        void testGetSet() {
            rp3d_test(mIdentityTransform.getPosition() == Vector3(0, 0, 0));
            rp3d_test(mIdentityTransform.getOrientation() == Quaternion::identity());
            Transform transform;
            transform.setPosition(Vector3(5, 7, 8));
            transform.setOrientation(Quaternion(1, 2, 3, 1));
            rp3d_test(transform.getPosition() == Vector3(5, 7, 8));
            rp3d_test(transform.getOrientation() == Quaternion(1, 2, 3, 1));
            transform.setToIdentity();
            rp3d_test(transform.getPosition() == Vector3(0, 0, 0));
            rp3d_test(transform.getOrientation() == Quaternion::identity());
        }

        /// Test the inverse
        void testInverse() {
            Transform inverseTransform = mTransform1.getInverse();
            Vector3 vector(2, 3, 4);
            Vector3 tempVector = mTransform1 * vector;
            Vector3 tempVector2 = inverseTransform * tempVector;
            rp3d_test(approxEqual(tempVector2.x, vector.x, decimal(10e-6)));
            rp3d_test(approxEqual(tempVector2.y, vector.y, decimal(10e-6)));
            rp3d_test(approxEqual(tempVector2.z, vector.z, decimal(10e-6)));
        }

        /// Test methods to set and get transform matrix from and to OpenGL
        void testGetSetOpenGLMatrix() {
            Transform transform;
            Vector3 position = mTransform1.getPosition();
            Matrix3x3 orientation = mTransform1.getOrientation().getMatrix();
            decimal openglMatrix[16] = {orientation[0][0], orientation[1][0],
                                        orientation[2][0], 0,
                                        orientation[0][1], orientation[1][1],
                                        orientation[2][1], 0,
                                        orientation[0][2], orientation[1][2],
                                        orientation[2][2], 0,
                                        position.x, position.y, position.z, 1};
            transform.setFromOpenGL(openglMatrix);
            decimal openglMatrix2[16];
            transform.getOpenGLMatrix(openglMatrix2);
            rp3d_test(approxEqual(openglMatrix2[0], orientation[0][0]));
            rp3d_test(approxEqual(openglMatrix2[1], orientation[1][0]));
            rp3d_test(approxEqual(openglMatrix2[2], orientation[2][0]));
            rp3d_test(approxEqual(openglMatrix2[3], 0));
            rp3d_test(approxEqual(openglMatrix2[4], orientation[0][1]));
            rp3d_test(approxEqual(openglMatrix2[5], orientation[1][1]));
            rp3d_test(approxEqual(openglMatrix2[6], orientation[2][1]));
            rp3d_test(approxEqual(openglMatrix2[7], 0));
            rp3d_test(approxEqual(openglMatrix2[8], orientation[0][2]));
            rp3d_test(approxEqual(openglMatrix2[9], orientation[1][2]));
            rp3d_test(approxEqual(openglMatrix2[10], orientation[2][2]));
            rp3d_test(approxEqual(openglMatrix2[11], 0));
            rp3d_test(approxEqual(openglMatrix2[12], position.x));
            rp3d_test(approxEqual(openglMatrix2[13], position.y));
            rp3d_test(approxEqual(openglMatrix2[14], position.z));
            rp3d_test(approxEqual(openglMatrix2[15], 1));
        }

        /// Test the method to interpolate transforms
        void testInterpolateTransform() {
            Transform transformStart = Transform::interpolateTransforms(mTransform1, mTransform2,0);
            Transform transformEnd = Transform::interpolateTransforms(mTransform1, mTransform2,1);
            rp3d_test(transformStart == mTransform1);
            rp3d_test(transformEnd == mTransform2);

            decimal sinA = sin(PI/3.0f);
            decimal cosA = cos(PI/3.0f);
            decimal sinB = sin(PI/6.0f);
            decimal cosB = cos(PI/6.0f);
            Transform transform1(Vector3(4, 5, 6), Quaternion::identity());
            Transform transform2(Vector3(8, 11, 16), Quaternion(sinA, sinA, sinA, cosA));
            Transform transform = Transform::interpolateTransforms(transform1, transform2, 0.5);
            Vector3 position = transform.getPosition();
            Quaternion orientation = transform.getOrientation();
            rp3d_test(approxEqual(position.x, 6));
            rp3d_test(approxEqual(position.y, 8));
            rp3d_test(approxEqual(position.z, 11));
            rp3d_test(approxEqual(orientation.x, sinB));
            rp3d_test(approxEqual(orientation.y, sinB));
            rp3d_test(approxEqual(orientation.z, sinB));
            rp3d_test(approxEqual(orientation.w, cosB));
        }

        /// Test the identity methods
        void testIdentity() {
            Transform transform = Transform::identity();
            rp3d_test(transform.getPosition() == Vector3(0, 0, 0));
            rp3d_test(transform.getOrientation() == Quaternion::identity());

            Transform transform2(Vector3(5, 6, 2), Quaternion(3, 5, 1, 6));
            transform2.setToIdentity();
            rp3d_test(transform2.getPosition() == Vector3(0, 0, 0));
            rp3d_test(transform2.getOrientation() == Quaternion::identity());
        }

        /// Test the overloaded operators
        void testOperators() {

            // Equality, inequality operator
            rp3d_test(mTransform1 == mTransform1);
            rp3d_test(mTransform1 != mTransform2);

            // Assignment operator
            Transform transform;
            transform = mTransform1;
            rp3d_test(transform == mTransform1);

            // Multiplication
            Vector3 vector(7, 53, 5);
            Vector3 vector2 = mTransform2 * (mTransform1 * vector);
            Vector3 vector3 = (mTransform2 * mTransform1) * vector;
            rp3d_test(approxEqual(vector2.x, vector3.x, decimal(10e-6)));
            rp3d_test(approxEqual(vector2.y, vector3.y, decimal(10e-6)));
            rp3d_test(approxEqual(vector2.z, vector3.z, decimal(10e-6)));
        }
 };

}

#endif
