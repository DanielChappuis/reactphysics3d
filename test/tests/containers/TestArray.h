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

#ifndef TEST_ARRAY_H
#define TEST_ARRAY_H

// Libraries
#include "Test.h"
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/memory/DefaultAllocator.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestArray
/**
 * Unit test for the Array class
 */
class TestArray : public Test {

    private :

        // ---------- Atributes ---------- //

        DefaultAllocator mAllocator;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestArray(const std::string& name) : Test(name) {

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

            Array<int> array1(mAllocator);
            rp3d_test(array1.capacity() == 0);
            rp3d_test(array1.size() == 0);

            Array<int> array2(mAllocator, 100);
            rp3d_test(array2.size() == 0);

            Array<int> array3(mAllocator);
            array3.add(1);
            array3.add(2);
            array3.add(3);
            rp3d_test(array3.size() == 3);

            // ----- Copy Constructors ----- //

            Array<int> array4(array1);
            rp3d_test(array4.capacity() == 0);
            rp3d_test(array4.size() == 0);

            Array<int> array5(array3);
            rp3d_test(array5.capacity() == array3.capacity());
            rp3d_test(array5.size() == array3.size());
            for (uint32 i=0; i<array3.size(); i++) {
                rp3d_test(array5[i] == array3[i]);
            }

            // ----- Test capacity grow ----- //
            Array<std::string> arra6(mAllocator, 20);
            for (uint32 i=0; i<20; i++) {
                arra6.add("test");
            }
            arra6.add("test");
        }

        void testAddRemoveClear() {

            // ----- Test add() ----- //

            Array<int> array1(mAllocator);
            array1.add(4);
            rp3d_test(array1.size() == 1);
            rp3d_test(array1[0] == 4);
            array1.add(9);
            rp3d_test(array1.size() == 2);
            rp3d_test(array1[0] == 4);
            rp3d_test(array1[1] == 9);

            const int arraySize = 15;
            int arrayTest[arraySize] = {3, 145, -182, 34, 12, 95, -1834, 4143, -111, -111, 4343, 234, 22983, -3432, 753};
            Array<int> array2(mAllocator);
            for (uint32 i=0; i<arraySize; i++) {
               array2.add(arrayTest[i]);
            }
            rp3d_test(array2.size() == arraySize);
            for (uint32 i=0; i<arraySize; i++) {
                rp3d_test(array2[i] == arrayTest[i]);
            }

            // ----- Test remove() ----- //

            Array<int> array3(mAllocator);
            array3.add(1);
            array3.add(2);
            array3.add(3);
            array3.add(4);

            auto it = array3.removeAt(3);
            rp3d_test(array3.size() == 3);
            rp3d_test(it == array3.end());
            rp3d_test(array3[0] == 1);
            rp3d_test(array3[1] == 2);
            rp3d_test(array3[2] == 3);

            it = array3.removeAt(1);
            rp3d_test(array3.size() == 2);
            rp3d_test(array3[0] == 1);
            rp3d_test(array3[1] == 3);
            rp3d_test(*it == 3);

            array3.removeAt(0);
            rp3d_test(array3.size() == 1);
            rp3d_test(array3[0] == 3);

            it = array3.removeAt(0);
            rp3d_test(array3.size() == 0);
            rp3d_test(it == array3.end());

            array3.add(1);
            array3.add(2);
            array3.add(3);
            it = array3.begin();
            array3.remove(it);
            rp3d_test(array3.size() == 2);
            rp3d_test(array3[0] == 2);
            rp3d_test(array3[1] == 3);
            it = array3.find(3);
            array3.remove(it);
            rp3d_test(array3.size() == 1);
            rp3d_test(array3[0] == 2);

            array3.add(5);
            array3.add(6);
            array3.add(7);
            it = array3.remove(7);
            rp3d_test(it == array3.end());
            rp3d_test(array3.size() == 3);
            it = array3.remove(5);
            rp3d_test((*it) == 6);

            // ----- Test addRange() ----- //

            Array<int> array4(mAllocator);
            array4.add(1);
            array4.add(2);
            array4.add(3);

            Array<int> array5(mAllocator);
            array5.add(4);
            array5.add(5);

            Array<int> array6(mAllocator);
            array6.addRange(array5);
            rp3d_test(array6.size() == array5.size());
            rp3d_test(array6[0] == 4);
            rp3d_test(array6[1] == 5);

            array4.addRange(array5);
            rp3d_test(array4.size() == 3 + array5.size());
            rp3d_test(array4[0] == 1);
            rp3d_test(array4[1] == 2);
            rp3d_test(array4[2] == 3);
            rp3d_test(array4[3] == 4);
            rp3d_test(array4[4] == 5);

            // ----- Test clear() ----- //

            Array<std::string> array7(mAllocator);
            array7.add("test1");
            array7.add("test2");
            array7.add("test3");
            array7.clear();
            rp3d_test(array7.size() == 0);
            array7.add("new");
            rp3d_test(array7.size() == 1);
            rp3d_test(array7[0] == "new");

            // ----- Test removeAtAndReplaceByLast() ----- //

            Array<int> array8(mAllocator);
            array8.add(1);
            array8.add(2);
            array8.add(3);
            array8.add(4);
            array8.removeAtAndReplaceByLast(1);
            rp3d_test(array8.size() == 3);
            rp3d_test(array8[0] == 1);
            rp3d_test(array8[1] == 4);
            rp3d_test(array8[2] == 3);
            array8.removeAtAndReplaceByLast(2);
            rp3d_test(array8.size() == 2);
            rp3d_test(array8[0] == 1);
            rp3d_test(array8[1] == 4);
            array8.removeAtAndReplaceByLast(0);
            rp3d_test(array8.size() == 1);
            rp3d_test(array8[0] == 4);
            array8.removeAtAndReplaceByLast(0);
            rp3d_test(array8.size() == 0);
        }

        void testAssignment() {

            Array<int> array1(mAllocator);
            array1.add(1);
            array1.add(2);
            array1.add(3);

            Array<int> array2(mAllocator);
            array2.add(5);
            array2.add(6);

            Array<int> array3(mAllocator);
            Array<int> array4(mAllocator);
            array4.add(1);
            array4.add(2);

            Array<int> array5(mAllocator);
            array5.add(1);
            array5.add(2);
            array5.add(3);

            array3 = array2;
            rp3d_test(array2.size() == array3.size());
            rp3d_test(array2[0] == array3[0]);
            rp3d_test(array2[1] == array3[1]);

            array4 = array1;
            rp3d_test(array4.size() == array1.size())
            rp3d_test(array4[0] == array1[0]);
            rp3d_test(array4[1] == array1[1]);
            rp3d_test(array4[2] == array1[2]);

            array5 = array2;
            rp3d_test(array5.size() == array2.size());
            rp3d_test(array5[0] == array2[0]);
            rp3d_test(array5[1] == array2[1]);
        }

        void testIndexing() {

            Array<int> array1(mAllocator);
            array1.add(1);
            array1.add(2);
            array1.add(3);

            rp3d_test(array1[0] == 1);
            rp3d_test(array1[1] == 2);
            rp3d_test(array1[2] == 3);

            array1[0] = 6;
            array1[1] = 7;
            array1[2] = 8;

            rp3d_test(array1[0] == 6);
            rp3d_test(array1[1] == 7);
            rp3d_test(array1[2] == 8);

            const int a = array1[0];
            const int b = array1[1];
            rp3d_test(a == 6);
            rp3d_test(b == 7);

            array1[0]++;
            array1[1]++;
            rp3d_test(array1[0] == 7);
            rp3d_test(array1[1] == 8);
        }

        void testFind() {

            Array<int> array1(mAllocator);
            array1.add(1);
            array1.add(2);
            array1.add(3);
            array1.add(4);
            array1.add(5);

            rp3d_test(array1.find(1) == array1.begin());
            rp3d_test(*(array1.find(2)) == 2);
            rp3d_test(*(array1.find(5)) == 5);
        }

        void testEquality() {

            Array<int> array1(mAllocator);
            array1.add(1);
            array1.add(2);
            array1.add(3);

            Array<int> array2(mAllocator);
            array2.add(1);
            array2.add(2);

            Array<int> array3(mAllocator);
            array3.add(1);
            array3.add(2);
            array3.add(3);

            Array<int> array4(mAllocator);
            array4.add(1);
            array4.add(5);
            array4.add(3);

            rp3d_test(array1 == array1);
            rp3d_test(array1 != array2);
            rp3d_test(array1 == array3);
            rp3d_test(array1 != array4);
            rp3d_test(array2 != array4);
        }

        void testReserve() {

            Array<int> array1(mAllocator);
            array1.reserve(10);
            rp3d_test(array1.size() == 0);
            array1.add(1);
            array1.add(2);
            rp3d_test(array1.size() == 2);
            rp3d_test(array1[0] == 1);
            rp3d_test(array1[1] == 2);

            array1.reserve(1);

            array1.reserve(100);
            rp3d_test(array1[0] == 1);
            rp3d_test(array1[1] == 2);
        }

        void testIterators() {

            Array<int> array1(mAllocator);

            rp3d_test(array1.begin() == array1.end());

            array1.add(5);
            array1.add(6);
            array1.add(8);
            array1.add(-1);

            Array<int>::Iterator itBegin = array1.begin();
            Array<int>::Iterator itEnd = array1.end();
            Array<int>::Iterator it = array1.begin();

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

            it = array1.end();
            rp3d_test(it == itEnd);
            it--;
            rp3d_test(*it == -1);
            it++;
            rp3d_test(it == itEnd);

            Array<int> array2(mAllocator);
            for (auto it = array1.begin(); it != array1.end(); ++it) {
                array2.add(*it);
            }

            rp3d_test(array1 == array2);

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
