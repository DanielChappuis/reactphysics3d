/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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

#ifndef TEST_DYNAMIC_AABB_TREE_H
#define TEST_DYNAMIC_AABB_TREE_H

// Libraries
#include "Test.h"
#include "collision/broadphase/DynamicAABBTree.h"

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestDynamicAABBTree
/**
 * Unit test for the dynamic AABB tree
 */
class TestDynamicAABBTree : public Test {

    private :

        // ---------- Atributes ---------- //

        // Dynamic AABB Tree
        DynamicAABBTree mTree;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestDynamicAABBTree(const std::string& name): Test(name)  {}

        /// Run the tests
        void run() {

            // TODO : Implement tests here
        }

 };

}

#endif
