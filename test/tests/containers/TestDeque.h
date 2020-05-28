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

#ifndef TEST_DEQUE_H
#define TEST_DEQUE_H

// Libraries
#include "Test.h"
#include <reactphysics3d/containers/Deque.h>
#include <reactphysics3d/memory/DefaultAllocator.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestDeque
/**
 * Unit test for the Deque class
 */
class TestDeque : public Test {

    private :

        // ---------- Atributes ---------- //

        DefaultAllocator mAllocator;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestDeque(const std::string& name) : Test(name) {

        }

        /// Run the tests
        void run() {

            testConstructors();
            testAddPopClear();
            testAssignment();
            testEquality();
            testIterators();
        }

        void testConstructors() {

            // ----- Constructors ----- //

            Deque<int> deque1(mAllocator);
            rp3d_test(deque1.size() == 0);

            Deque<int> deque2(mAllocator);
            deque2.addBack(1);
            deque2.addBack(2);
            deque2.addBack(3);
            rp3d_test(deque2.size() == 3);

            // ----- Copy Constructors ----- //

            Deque<int> deque3(deque1);
            rp3d_test(deque3.size() == 0);

            Deque<int> deque4(deque2);
            rp3d_test(deque4.size() == deque2.size());
            Deque<int>::Iterator it3 = deque2.begin();
            Deque<int>::Iterator it5 = deque4.begin();
            for (uint i=0; i<deque2.size(); i++) {
                rp3d_test((*it3) == (*it5));
                ++it3;
                ++it5;
            }

            // ----- Test capacity grow ----- //
            Deque<std::string> deque5(mAllocator);
            for (uint i=0; i<300; i++) {
                deque5.addBack("test");
            }

            // ----- Test [] operator ----- //
            Deque<int> deque6(deque2);
            deque6.addBack(1);
            rp3d_test(deque6[0] == 1);
            deque6.addBack(2);
            rp3d_test(deque6[0] == 1);
            rp3d_test(deque6[1] == 2);
            deque6.addFront(5);
            rp3d_test(deque6[0] == 5);
            rp3d_test(deque6[1] == 1);
            rp3d_test(deque6[2] == 2);
            deque6.addFront(7);
            rp3d_test(deque6[0] == 7);
            rp3d_test(deque6[1] == 5);
            rp3d_test(deque6[2] == 1);
            rp3d_test(deque6[3] == 2);
            deque6.popFront();
            rp3d_test(deque6[0] == 5);
            rp3d_test(deque6[1] == 1);
            rp3d_test(deque6[2] == 2);
        }

        void testAddPopClear() {

            // ----- Test addBack() ----- //

            Deque<int> deque1(mAllocator);
            deque1.addBack(4);
            rp3d_test(deque1.size() == 1);
            rp3d_test(deque1.getBack() == 4);
            rp3d_test(deque1.getFront() == 4);
            deque1.addBack(9);
            rp3d_test(deque1.size() == 2);
            rp3d_test(deque1.getBack() == 9);
            rp3d_test(deque1.getFront() == 4);
            deque1.addBack(1);
            rp3d_test(deque1.size() == 3);
            rp3d_test(deque1.getBack() == 1);
            rp3d_test(deque1.getFront() == 4);

            // ----- Test addFront() ----- //

            Deque<int> deque2(mAllocator);
            deque2.addFront(4);
            rp3d_test(deque2.size() == 1);
            rp3d_test(deque2.getBack() == 4);
            rp3d_test(deque2.getFront() == 4);
            deque2.addFront(9);
            rp3d_test(deque2.size() == 2);
            rp3d_test(deque2.getBack() == 4);
            rp3d_test(deque2.getFront() == 9);
            deque2.addFront(1);
            rp3d_test(deque2.size() == 3);
            rp3d_test(deque2.getBack() == 4);
            rp3d_test(deque2.getFront() == 1);

            // ----- Test popFront() ----- //

            deque1.popFront();
            rp3d_test(deque1.size() == 2);
            rp3d_test(deque1.getFront() == 9);
            deque1.popFront();
            rp3d_test(deque1.size() == 1);
            rp3d_test(deque1.getFront() == 1);
            deque1.popFront();
            rp3d_test(deque1.size() == 0);

            // ----- Test popBack() ----- //

            deque2.popBack();
            rp3d_test(deque2.size() == 2);
            rp3d_test(deque2.getBack() == 9);
            deque2.popBack();
            rp3d_test(deque2.size() == 1);
            rp3d_test(deque2.getBack() == 1);
            deque2.popBack();
            rp3d_test(deque2.size() == 0);

            deque2.addBack(1);
            deque2.addFront(2);
            rp3d_test(deque2.getBack() == 1);
            rp3d_test(deque2.getFront() == 2);
            deque2.addBack(3);
            deque2.addFront(4);
            rp3d_test(deque2.getBack() == 3);
            rp3d_test(deque2.getFront() == 4);
            deque2.popBack();
            deque2.popFront();
            rp3d_test(deque2.getBack() == 1);
            rp3d_test(deque2.getFront() == 2);

            // ----- Test clear() ----- //

            deque1.clear();
            deque2.clear();
            rp3d_test(deque1.size() == 0);
            rp3d_test(deque2.size() == 0);

            deque1.addBack(1);
            deque1.addFront(2);
            deque1.addBack(3);
            deque1.addFront(4);
            rp3d_test(deque1.getBack() == 3);
            rp3d_test(deque1.getFront() == 4);
            deque1.clear();
            rp3d_test(deque1.size() == 0);

        }

        void testAssignment() {

            Deque<int> deque1(mAllocator);
            deque1.addBack(1);
            deque1.addBack(2);
            deque1.addBack(3);

            Deque<int> deque2(mAllocator);
            deque2.addFront(5);
            deque2.addBack(6);

            Deque<int> deque3(mAllocator);
            Deque<int> deque4(mAllocator);
            deque4.addBack(7);
            deque4.addBack(9);

            deque3 = deque2;
            rp3d_test(deque2.size() == deque3.size());
            rp3d_test(deque2.size() == 2);
            rp3d_test(deque2.getBack() == deque3.getBack());
            rp3d_test(deque2.getFront() == deque3.getFront());

            deque4 = deque1;
            rp3d_test(deque4.size() == deque1.size())
            rp3d_test(deque4.size() == 3)
            rp3d_test(deque4.getFront() == 1);
            rp3d_test(deque4.getBack() == 3);
        }

        void testEquality() {

            Deque<int> deque1(mAllocator);
            deque1.addFront(1);
            deque1.addBack(2);
            deque1.addBack(3);

            Deque<int> deque2(mAllocator);
            deque2.addBack(1);
            deque2.addBack(2);

            Deque<int> deque3(mAllocator);
            deque3.addBack(1);
            deque3.addBack(2);
            deque3.addBack(3);

            Deque<int> deque4(mAllocator);
            deque4.addFront(1);
            deque4.addFront(5);
            deque4.addFront(3);

            Deque<int> deque5(mAllocator);
            deque5.addFront(3);
            deque5.addFront(2);
            deque5.addFront(1);

            rp3d_test(deque1 == deque1);
            rp3d_test(deque1 != deque2);
            rp3d_test(deque1 == deque3);
            rp3d_test(deque1 != deque4);
            rp3d_test(deque2 != deque4);
            rp3d_test(deque1 == deque5);
        }

        void testIterators() {

            Deque<int> deque1(mAllocator);

            rp3d_test(deque1.begin() == deque1.end());

            deque1.addBack(5);
            deque1.addBack(6);
            deque1.addBack(8);
            deque1.addBack(-1);

            Deque<int>::Iterator itBegin = deque1.begin();
            Deque<int>::Iterator itEnd = deque1.end();
            Deque<int>::Iterator it = deque1.begin();

            rp3d_test(itBegin < itEnd);
            rp3d_test(itBegin <= itEnd);
            rp3d_test(itEnd > itBegin);
            rp3d_test(itEnd >= itBegin);

            rp3d_test(itBegin == it);
            rp3d_test(*it == 5);
            rp3d_test(*(it++) == 5);
            rp3d_test(*it == 6);
            rp3d_test(*(it--) == 6);
            rp3d_test(*it == 5);
            rp3d_test(*(++it) == 6);
            rp3d_test(*it == 6);
            rp3d_test(*(--it) == 5);
            rp3d_test(*it == 5);
            rp3d_test(it == itBegin);

            it = deque1.end();
            rp3d_test(it == itEnd);
            it--;
            rp3d_test(*it == -1);
            it++;
            rp3d_test(it == itEnd);

            Deque<int> deque2(mAllocator);
            for (auto it = deque1.begin(); it != deque1.end(); ++it) {
                deque2.addBack(*it);
            }

            rp3d_test(deque1 == deque2);

            it = itBegin;
            rp3d_test(*(it + 2) == 8);
            it += 2;
            rp3d_test(*it == 8);
            rp3d_test(*(it - 2) == 5);
            it -= 2;
            rp3d_test(*it == 5);
            rp3d_test((itEnd - itBegin) == 4);

            it = itBegin;
            *it = 19;
            rp3d_test(*it == 19);
        }

 };

}

#endif
