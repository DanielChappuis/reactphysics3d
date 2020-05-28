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

#ifndef TEST_STACK_H
#define TEST_STACK_H

// Libraries
#include "Test.h"
#include <reactphysics3d/containers/Stack.h>
#include <reactphysics3d/memory/DefaultAllocator.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestStack
/**
 * Unit test for the Stack class
 */
class TestStack : public Test {

    private :

        // ---------- Atributes ---------- //

        DefaultAllocator mAllocator;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestStack(const std::string& name) : Test(name) {

        }

        /// Run the tests
        void run() {

            testConstructor();
            testPushPop();
        }

        void testConstructor() {

            // ----- Constructors ----- //

            Stack<std::string> stack1(mAllocator);
            rp3d_test(stack1.capacity() == 0);
            rp3d_test(stack1.size() == 0);

            Stack<std::string> stack2(mAllocator, 100);
            rp3d_test(stack2.capacity() >= 100);
            rp3d_test(stack2.size() == 0);

            // ----- Copy Constructors ----- //
            Stack<std::string> stack3(stack2);
            rp3d_test(stack3.capacity() == stack2.capacity());
            rp3d_test(stack3.size() == stack2.size());

            Stack<int> stack4(mAllocator);
            stack4.push(10);
            stack4.push(20);
            stack4.push(30);
            rp3d_test(stack4.capacity() >= 3);
            rp3d_test(stack4.size() == 3);
            stack4.push(40);
            rp3d_test(stack4.size() == 4);

            Stack<int> set5(stack4);
            rp3d_test(set5.capacity() == stack4.capacity());
            rp3d_test(set5.size() == stack4.size());
            rp3d_test(set5.pop() == 40);
            rp3d_test(set5.size() == 3);
            rp3d_test(set5.pop() == 30);
            rp3d_test(set5.pop() == 20);
            rp3d_test(set5.pop() == 10);
            rp3d_test(set5.size() == 0);
        }

        void testPushPop() {

            Stack<int> stack1(mAllocator);
            stack1.push(10);
            stack1.push(80);
            stack1.push(130);
            rp3d_test(stack1.size() == 3);
            rp3d_test(stack1.pop() == 130);
            rp3d_test(stack1.pop() == 80);
            rp3d_test(stack1.pop() == 10);
            rp3d_test(stack1.size() == 0);
            stack1.push(10);
            rp3d_test(stack1.pop() == 10);
            stack1.push(10);
            stack1.push(80);
            stack1.pop();
            rp3d_test(stack1.pop() == 10);
            rp3d_test(stack1.size() == 0);
            stack1.push(10);
            stack1.push(80);
            stack1.push(130);
            stack1.push(56);
            stack1.push(89);
            stack1.push(131);
            stack1.clear();
            rp3d_test(stack1.size() == 0);
        }
 };

}

#endif
