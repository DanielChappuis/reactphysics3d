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

#ifndef TEST_LIST_H
#define TEST_LIST_H

// Libraries
#include "Test.h"
#include <reactphysics3d/containers/List.h>
#include <reactphysics3d/memory/DefaultAllocator.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestList
/**
 * Unit test for the List class
 */
class TestList : public Test {

    private :

        // ---------- Atributes ---------- //

        DefaultAllocator mAllocator;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestList(const std::string& name) : Test(name) {

        }

        /// Run the tests
        void run() {

            testConstructors();
            testAddRemoveClear();
            testAssignment();
            testIndexing();
            testFind();
            testEquality();
            testReserve();
            testIterators();
        }

        void testConstructors() {

            // ----- Constructors ----- //

            List<int> list1(mAllocator);
            rp3d_test(list1.capacity() == 0);
            rp3d_test(list1.size() == 0);

            List<int> list2(mAllocator, 100);
            rp3d_test(list2.capacity() == 100);
            rp3d_test(list2.size() == 0);

            List<int> list3(mAllocator);
            list3.add(1);
            list3.add(2);
            list3.add(3);
            rp3d_test(list3.capacity() == 4);
            rp3d_test(list3.size() == 3);

            // ----- Copy Constructors ----- //

            List<int> list4(list1);
            rp3d_test(list4.capacity() == 0);
            rp3d_test(list4.size() == 0);

            List<int> list5(list3);
            rp3d_test(list5.capacity() == list3.size());
            rp3d_test(list5.size() == list3.size());
            for (uint i=0; i<list3.size(); i++) {
                rp3d_test(list5[i] == list3[i]);
            }

            // ----- Test capacity grow ----- //
            List<std::string> list6(mAllocator, 20);
            rp3d_test(list6.capacity() == 20);
            for (uint i=0; i<20; i++) {
                list6.add("test");
            }
            rp3d_test(list6.capacity() == 20);
            list6.add("test");
            rp3d_test(list6.capacity() == 40);
        }

        void testAddRemoveClear() {

            // ----- Test add() ----- //

            List<int> list1(mAllocator);
            list1.add(4);
            rp3d_test(list1.size() == 1);
            rp3d_test(list1[0] == 4);
            list1.add(9);
            rp3d_test(list1.size() == 2);
            rp3d_test(list1[0] == 4);
            rp3d_test(list1[1] == 9);

            const int arraySize = 15;
            int arrayTest[arraySize] = {3, 145, -182, 34, 12, 95, -1834, 4143, -111, -111, 4343, 234, 22983, -3432, 753};
            List<int> list2(mAllocator);
            for (uint i=0; i<arraySize; i++) {
               list2.add(arrayTest[i]);
            }
            rp3d_test(list2.size() == arraySize);
            for (uint i=0; i<arraySize; i++) {
                rp3d_test(list2[i] == arrayTest[i]);
            }

            // ----- Test remove() ----- //

            List<int> list3(mAllocator);
            list3.add(1);
            list3.add(2);
            list3.add(3);
            list3.add(4);

            auto it = list3.removeAt(3);
            rp3d_test(list3.size() == 3);
            rp3d_test(list3.capacity() == 4);
            rp3d_test(it == list3.end());
            rp3d_test(list3[0] == 1);
            rp3d_test(list3[1] == 2);
            rp3d_test(list3[2] == 3);

            it = list3.removeAt(1);
            rp3d_test(list3.size() == 2);
            rp3d_test(list3.capacity() == 4);
            rp3d_test(list3[0] == 1);
            rp3d_test(list3[1] == 3);
            rp3d_test(*it == 3);

            list3.removeAt(0);
            rp3d_test(list3.size() == 1);
            rp3d_test(list3.capacity() == 4);
            rp3d_test(list3[0] == 3);

            it = list3.removeAt(0);
            rp3d_test(list3.size() == 0);
            rp3d_test(list3.capacity() == 4);
            rp3d_test(it == list3.end());

            list3.add(1);
            list3.add(2);
            list3.add(3);
            it = list3.begin();
            list3.remove(it);
            rp3d_test(list3.size() == 2);
            rp3d_test(list3[0] == 2);
            rp3d_test(list3[1] == 3);
            it = list3.find(3);
            list3.remove(it);
            rp3d_test(list3.size() == 1);
            rp3d_test(list3[0] == 2);

            list3.add(5);
            list3.add(6);
            list3.add(7);
            it = list3.remove(7);
            rp3d_test(it == list3.end());
            rp3d_test(list3.size() == 3);
            it = list3.remove(5);
            rp3d_test((*it) == 6);

            // ----- Test addRange() ----- //

            List<int> list4(mAllocator);
            list4.add(1);
            list4.add(2);
            list4.add(3);

            List<int> list5(mAllocator);
            list5.add(4);
            list5.add(5);

            List<int> list6(mAllocator);
            list6.addRange(list5);
            rp3d_test(list6.size() == list5.size());
            rp3d_test(list6[0] == 4);
            rp3d_test(list6[1] == 5);

            list4.addRange(list5);
            rp3d_test(list4.size() == 3 + list5.size());
            rp3d_test(list4[0] == 1);
            rp3d_test(list4[1] == 2);
            rp3d_test(list4[2] == 3);
            rp3d_test(list4[3] == 4);
            rp3d_test(list4[4] == 5);

            // ----- Test clear() ----- //

            List<std::string> list7(mAllocator);
            list7.add("test1");
            list7.add("test2");
            list7.add("test3");
            list7.clear();
            rp3d_test(list7.size() == 0);
            list7.add("new");
            rp3d_test(list7.size() == 1);
            rp3d_test(list7[0] == "new");
        }

        void testAssignment() {

            List<int> list1(mAllocator);
            list1.add(1);
            list1.add(2);
            list1.add(3);

            List<int> list2(mAllocator);
            list2.add(5);
            list2.add(6);

            List<int> list3(mAllocator);
            List<int> list4(mAllocator);
            list4.add(1);
            list4.add(2);

            List<int> list5(mAllocator);
            list5.add(1);
            list5.add(2);
            list5.add(3);

            list3 = list2;
            rp3d_test(list2.size() == list3.size());
            rp3d_test(list2[0] == list3[0]);
            rp3d_test(list2[1] == list3[1]);

            list4 = list1;
            rp3d_test(list4.size() == list1.size())
            rp3d_test(list4[0] == list1[0]);
            rp3d_test(list4[1] == list1[1]);
            rp3d_test(list4[2] == list1[2]);

            list5 = list2;
            rp3d_test(list5.size() == list2.size());
            rp3d_test(list5[0] == list2[0]);
            rp3d_test(list5[1] == list2[1]);
        }

        void testIndexing() {

            List<int> list1(mAllocator);
            list1.add(1);
            list1.add(2);
            list1.add(3);

            rp3d_test(list1[0] == 1);
            rp3d_test(list1[1] == 2);
            rp3d_test(list1[2] == 3);

            list1[0] = 6;
            list1[1] = 7;
            list1[2] = 8;

            rp3d_test(list1[0] == 6);
            rp3d_test(list1[1] == 7);
            rp3d_test(list1[2] == 8);

            const int a = list1[0];
            const int b = list1[1];
            rp3d_test(a == 6);
            rp3d_test(b == 7);

            list1[0]++;
            list1[1]++;
            rp3d_test(list1[0] == 7);
            rp3d_test(list1[1] == 8);
        }

        void testFind() {

            List<int> list1(mAllocator);
            list1.add(1);
            list1.add(2);
            list1.add(3);
            list1.add(4);
            list1.add(5);

            rp3d_test(list1.find(1) == list1.begin());
            rp3d_test(*(list1.find(2)) == 2);
            rp3d_test(*(list1.find(5)) == 5);
        }

        void testEquality() {

            List<int> list1(mAllocator);
            list1.add(1);
            list1.add(2);
            list1.add(3);

            List<int> list2(mAllocator);
            list2.add(1);
            list2.add(2);

            List<int> list3(mAllocator);
            list3.add(1);
            list3.add(2);
            list3.add(3);

            List<int> list4(mAllocator);
            list4.add(1);
            list4.add(5);
            list4.add(3);

            rp3d_test(list1 == list1);
            rp3d_test(list1 != list2);
            rp3d_test(list1 == list3);
            rp3d_test(list1 != list4);
            rp3d_test(list2 != list4);
        }

        void testReserve() {

            List<int> list1(mAllocator);
            list1.reserve(10);
            rp3d_test(list1.size() == 0);
            rp3d_test(list1.capacity() == 10);
            list1.add(1);
            list1.add(2);
            rp3d_test(list1.capacity() == 10);
            rp3d_test(list1.size() == 2);
            rp3d_test(list1[0] == 1);
            rp3d_test(list1[1] == 2);

            list1.reserve(1);
            rp3d_test(list1.capacity() == 10);

            list1.reserve(100);
            rp3d_test(list1.capacity() == 100);
            rp3d_test(list1[0] == 1);
            rp3d_test(list1[1] == 2);
        }

        void testIterators() {

            List<int> list1(mAllocator);

            rp3d_test(list1.begin() == list1.end());

            list1.add(5);
            list1.add(6);
            list1.add(8);
            list1.add(-1);

            List<int>::Iterator itBegin = list1.begin();
            List<int>::Iterator itEnd = list1.end();
            List<int>::Iterator it = list1.begin();

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

            it = list1.end();
            rp3d_test(it == itEnd);
            it--;
            rp3d_test(*it == -1);
            it++;
            rp3d_test(it == itEnd);

            List<int> list2(mAllocator);
            for (auto it = list1.begin(); it != list1.end(); ++it) {
                list2.add(*it);
            }

            rp3d_test(list1 == list2);

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
