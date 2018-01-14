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

#ifndef TEST_MAP_H
#define TEST_MAP_H

// Libraries
#include "Test.h"
#include "containers/Map.h"
#include "memory/DefaultAllocator.h"

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestMap
/**
 * Unit test for the Map class
 */
class TestMap : public Test {

    private :

        // ---------- Atributes ---------- //

        DefaultAllocator mAllocator;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestMap(const std::string& name) : Test(name) {

        }

        /// Run the tests
        void run() {

            testConstructors();
            testReserve();
            testAddRemoveClear();
            testContainsKey();
            testIndexing();
            testEquality();
            testAssignment();
            testIteration();
        }

        void testConstructors() {

            // ----- Constructors ----- //

            Map<int, std::string> map1(mAllocator);
            test(map1.capacity() == 0);
            test(map1.size() == 0);

            Map<int, std::string> map2(mAllocator, 100);
            test(map2.capacity() >= 100);
            test(map2.size() == 0);

            // ----- Copy Constructors ----- //
/*
            Map<int, std::string> map3(map1);
            test(map3.capacity() == map1.capacity());
            test(map3.size() == map1.size());

            Map<int, int> map4(mAllocator);
            map4.add(std::make_pair(1, 10));
            map4.add(std::make_pair(2, 20));
            map4.add(std::make_pair(3, 30));
            test(map4.capacity() >= 3);
            test(map4.size() == 3);

            Map<int, int> map5(map4);
            test(map5.capacity() == map4.capacity());
            test(map5.size() == map4.size());
            test(map5[1] == 10);
            test(map5[2] == 20);
            test(map5[3] == 30);
            */
        }

        void testReserve() {

            Map<int, std::string> map1(mAllocator);
            map1.reserve(15);
            test(map1.capacity() >= 15);
            map1.add(std::make_pair(1, "test1"));
            map1.add(std::make_pair(2, "test2"));
            test(map1.capacity() >= 15);

            map1.reserve(10);
            test(map1.capacity() >= 15);

            map1.reserve(100);
            test(map1.capacity() >= 100);
            test(map1[1] == "test1");
            test(map1[2] == "test2");
        }

        void testAddRemoveClear() {

            // ----- Test add() ----- //

            Map<int, int> map1(mAllocator);
            map1.add(std::make_pair(1, 10));
            map1.add(std::make_pair(8, 80));
            map1.add(std::make_pair(13, 130));
            test(map1[1] == 10);
            test(map1[8] == 80);
            test(map1[13] == 130);
            test(map1.size() == 3);

            Map<int, int> map2(mAllocator, 15);
            for (int i = 0; i < 1000000; i++) {
                map2.add(std::make_pair(i, i * 100));
            }
            bool isValid = true;
            for (int i = 0; i < 1000000; i++) {
                if (map2[i] != i * 100) isValid = false;
            }
            test(isValid);

            map1.remove(1);
            map1.add(std::make_pair(1, 10));
            test(map1.size() == 3);
            test(map1[1] == 10);

            // ----- Test remove() ----- //

            map1.remove(1);
            test(!map1.containsKey(1));
            test(map1.containsKey(8));
            test(map1.containsKey(13));
            test(map1.size() == 2);

            map1.remove(13);
            test(!map1.containsKey(8));
            test(map1.containsKey(13));
            test(map1.size() == 1);

            map1.remove(8);
            test(!map1.containsKey(8));
            test(map1.size() == 0);

            isValid = true;
            for (int i = 0; i < 1000000; i++) {
                map2.remove(i);
            }
            for (int i = 0; i < 1000000; i++) {
                if (map2.containsKey(i)) isValid = false;
            }
            test(isValid);
            test(map2.size() == 0);

            Map<int, int> map3(mAllocator);
            for (int i=0; i < 1000000; i++) {
                map3.add(std::make_pair(i, i * 10));
                map3.remove(i);
            }

            // ----- Test clear() ----- //

            Map<int, int> map4(mAllocator);
            map4.add(std::make_pair(2, 20));
            map4.add(std::make_pair(4, 40));
            map4.add(std::make_pair(6, 60));
            map4.clear();
            test(map4.size() == 0);
            map4.add(std::make_pair(2, 20));
            test(map4.size() == 1);
            test(map4[2] == 20);
            map4.clear();
            test(map4.size() == 0);

            Map<int, int> map5(mAllocator);
            map5.clear();
            test(map5.size() == 0);
        }

        void testContainsKey() {

            Map<int, int> map1(mAllocator);

            test(!map1.containsKey(2));
            test(!map1.containsKey(4));
            test(!map1.containsKey(6));

            map1.add(std::make_pair(2, 20));
            map1.add(std::make_pair(4, 40));
            map1.add(std::make_pair(6, 60));

            test(map1.containsKey(2));
            test(map1.containsKey(4));
            test(map1.containsKey(6));

            map1.remove(4);
            test(!map1.containsKey(4));
            test(map1.containsKey(2));
            test(map1.containsKey(6));

            map1.clear();
            test(!map1.containsKey(2));
            test(!map1.containsKey(6));
        }

        void testIndexing() {

            Map<int, int> map1(mAllocator);
            map1.add(std::make_pair(2, 20));
            map1.add(std::make_pair(4, 40));
            map1.add(std::make_pair(6, 60));
            test(map1[2] == 20);
            test(map1[4] == 40);
            test(map1[6] == 60);

            map1[2] = 10;
            map1[4] = 20;
            map1[6] = 30;

            test(map1[2] == 10);
            test(map1[4] == 20);
            test(map1[6] == 30);
        }

        void testEquality() {

            Map<std::string, int> map1(mAllocator, 10);
            Map<std::string, int> map2(mAllocator, 2);

            test(map1 == map2);

            map1.add(std::make_pair("a", 1));
            map1.add(std::make_pair("b", 2));
            map1.add(std::make_pair("c", 3));

            map2.add(std::make_pair("a", 1));
            map2.add(std::make_pair("b", 2));
            map2.add(std::make_pair("c", 4));

            test(map1 == map1);
            test(map2 == map2);
            test(map1 != map2);

            map2["c"] = 3;

            test(map1 == map2);

            Map<std::string, int> map3(mAllocator);
            map3.add(std::make_pair("a", 1));

            test(map1 != map3);
            test(map2 != map3);
        }

        void testAssignment() {

           Map<int, int> map1(mAllocator);
           map1.add(std::make_pair(1, 3));
           map1.add(std::make_pair(2, 6));
           map1.add(std::make_pair(10, 30));
/*
           Map<int, int> map2(mAllocator);
           map2 = map1;
           test(map2.size() == map1.size());
           test(map2[1] == 3);
           test(map2[2] == 6);
           test(map2[10] == 30);

           Map<int, int> map3(mAllocator, 100);
           map3 = map1;
           test(map3.size() == map1.size());
           test(map3[1] == 3);
           test(map3[2] == 6);
           test(map3[10] == 30);

           Map<int, int> map4(mAllocator);
           map3 = map4;
           test(map3.size() == 0);
*/
           Map<int, int> map5(mAllocator);
           map5.add(std::make_pair(7, 8));
           map5.add(std::make_pair(19, 70));
           map1 = map5;
           test(map5.size() == map1.size());
           test(map1[7] == 8);
           test(map1[19] == 70);
        }

        void testIteration() {

        }
 };

}

#endif
