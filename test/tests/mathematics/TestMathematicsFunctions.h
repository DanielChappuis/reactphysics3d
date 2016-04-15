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

#ifndef TEST_MATHEMATICS_FUNCTIONS_H
#define TEST_MATHEMATICS_FUNCTIONS_H

// Libraries
#include "Test.h"
#include "mathematics/mathematics_functions.h"

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestMathematicsFunctions
/**
 * Unit test for mathematics functions
 */
class TestMathematicsFunctions : public Test {

    private :

        // ---------- Atributes ---------- //



    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestMathematicsFunctions(const std::string& name): Test(name)  {}

        /// Run the tests
        void run() {

            // Test approxEqual()
            test(approxEqual(2, 7, 5.2));
            test(approxEqual(7, 2, 5.2));
            test(approxEqual(6, 6));
            test(!approxEqual(1, 5));
            test(!approxEqual(1, 5, 3));
            test(approxEqual(-2, -2));
            test(approxEqual(-2, -7, 6));
            test(!approxEqual(-2, 7, 2));
            test(approxEqual(-3, 8, 12));
            test(!approxEqual(-3, 8, 6));

            // Test clamp()
            test(clamp(4, -3, 5) == 4);
            test(clamp(-3, 1, 8) == 1);
            test(clamp(45, -6, 7) == 7);
            test(clamp(-5, -2, -1) == -2);
            test(clamp(-5, -9, -1) == -5);
            test(clamp(6, 6, 9) == 6);
            test(clamp(9, 6, 9) == 9);
            test(clamp(decimal(4), decimal(-3), decimal(5)) == decimal(4));
            test(clamp(decimal(-3), decimal(1), decimal(8)) == decimal(1));
            test(clamp(decimal(45), decimal(-6), decimal(7)) == decimal(7));
            test(clamp(decimal(-5), decimal(-2), decimal(-1)) == decimal(-2));
            test(clamp(decimal(-5), decimal(-9), decimal(-1)) == decimal(-5));
            test(clamp(decimal(6), decimal(6), decimal(9)) == decimal(6));
            test(clamp(decimal(9), decimal(6), decimal(9)) == decimal(9));

            // Test min3()
            test(min3(1, 5, 7) == 1);
            test(min3(-4, 2, 4) == -4);
            test(min3(-1, -5, -7) == -7);
            test(min3(13, 5, 47) == 5);
            test(min3(4, 4, 4) == 4);

            // Test max3()
            test(max3(1, 5, 7) == 7);
            test(max3(-4, 2, 4) == 4);
            test(max3(-1, -5, -7) == -1);
            test(max3(13, 5, 47) == 47);
            test(max3(4, 4, 4) == 4);

            // Test sameSign()
            test(sameSign(4, 53));
            test(sameSign(-4, -8));
            test(!sameSign(4, -7));
            test(!sameSign(-4, 53));

            // Test computeBarycentricCoordinatesInTriangle()
            Vector3 a(0, 0, 0);
            Vector3 b(5, 0, 0);
            Vector3 c(0, 0, 5);
            Vector3 testPoint(4, 0, 1);
            decimal u,v,w;
            computeBarycentricCoordinatesInTriangle(a, b, c, a, u, v, w);
            test(approxEqual(u, 1.0, 0.000001));
            test(approxEqual(v, 0.0, 0.000001));
            test(approxEqual(w, 0.0, 0.000001));
            computeBarycentricCoordinatesInTriangle(a, b, c, b, u, v, w);
            test(approxEqual(u, 0.0, 0.000001));
            test(approxEqual(v, 1.0, 0.000001));
            test(approxEqual(w, 0.0, 0.000001));
            computeBarycentricCoordinatesInTriangle(a, b, c, c, u, v, w);
            test(approxEqual(u, 0.0, 0.000001));
            test(approxEqual(v, 0.0, 0.000001));
            test(approxEqual(w, 1.0, 0.000001));

            computeBarycentricCoordinatesInTriangle(a, b, c, testPoint, u, v, w);
            test(approxEqual(u + v + w, 1.0, 0.000001));
        }

 };

}

#endif
