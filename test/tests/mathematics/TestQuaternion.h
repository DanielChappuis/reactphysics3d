
/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef TEST_QUATERNION_H
#define TEST_QUATERNION_H

#endif

// Libraries
#include "../../Test.h"
#include "../../../src/mathematics/Quaternion.h"

using namespace reactphysics3d;

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestQuaternion
/**
 * Unit test for the Quaternion class
 */
class TestQuaternion : public Test {

    private :

        // ---------- Atributes ---------- //

        /// Identity Quaternion
        Quaternion mIdentity;

        /// First test quaternion
        Quaternion mQuaternion1;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestQuaternion() : mIdentity(Quaternion::identity()) {

            decimal sinA = sin(PI/8.0f);
            decimal cosA = cos(PI/8.0f);
            mQuaternion1 = Quaternion(sinA, sinA, sinA, cosA);
        }

        /// Run the tests
        void run() {
            testConstructors();
        }

        /// Test the constructors
        void testConstructors() {
            Quaternion quaternion1(mQuaternion1);
            test(mQuaternion1== quaternion1);

            Quaternion quaternion2(4, 5, 6, 7);
            test(quaternion2 == Quaternion(4, 5, 6, 7));

            Quaternion quaternion3(8, Vector3(3, 5, 2));
            test(quaternion3 == Quaternion(5, 6, 7, 4));

            Matrix3x3 matrix(2, 3, 4, 5, 6, 7, 8, 9, 10);
            Quaternion quaternion4(matrix);
            Matrix3x3 result = quaternion4.getMatrix();
            test(approxEqual(matrix.getValue(0, 0), result.getValue(0, 0), 0.1));
            std::cout << "matrix : " << matrix.getValue(0, 0) << ", " << result.getValue(0, 0) << std::endl;
            test(approxEqual(matrix.getValue(0, 1), result.getValue(0, 1), 0.1));
            test(approxEqual(matrix.getValue(0, 2), result.getValue(0, 2), 0.1));
            test(approxEqual(matrix.getValue(1, 0), result.getValue(1, 0), 0.1));
            test(approxEqual(matrix.getValue(1, 1), result.getValue(1, 1), 0.1));
            test(approxEqual(matrix.getValue(1, 2), result.getValue(1, 2), 0.1));
            test(approxEqual(matrix.getValue(2, 0), result.getValue(2, 0), 0.1));
            test(approxEqual(matrix.getValue(2, 1), result.getValue(2, 1), 0.1));
            test(approxEqual(matrix.getValue(2, 2), result.getValue(2, 2), 0.1));
        }
 };

}
