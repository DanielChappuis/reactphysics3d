
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

#ifndef TEST_MATRIX3X3_H
#define TEST_MATRIX3X3_H

#endif

// Libraries
#include "../../Test.h"
#include "../../../src/mathematics/Matrix3x3.h"

using namespace reactphysics3d;

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestMatrix3x3
/**
 * Unit test for the Matrix3x3 class
 */
class TestMatrix3x3 : public Test {

    private :

        // ---------- Atributes ---------- //

        /// Identity transform
        Matrix3x3 mIdentity;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestMatrix3x3() : mIdentity(Matrix3x3::identity()){


        }

        /// Run the tests
        void run() {

        }


 };

}
