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
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/memory/DefaultAllocator.h>

// Key to test map with always same hash values
namespace reactphysics3d {
    struct TestKey {
        int key;

        TestKey(int k) :key(k) {}

        bool operator==(const TestKey& testKey) const {
            return key == testKey.key;
        }
    };
}

// Hash function for struct VerticesPair
namespace std {

  template <> struct hash<reactphysics3d::TestKey> {

    size_t operator()(const reactphysics3d::TestKey& key) const {
        return 1;
    }
  };
}

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
            testFind();
            testIndexing();
            testEquality();
            testAssignment();
            testIterators();
        }

        void testConstructors() {

            // ----- Constructors ----- //

            Map<int, std::string> map1(mAllocator);
            rp3d_test(map1.capacity() == 0);
            rp3d_test(map1.size() == 0);

            Map<int, std::string> map2(mAllocator, 100);
            rp3d_test(map2.capacity() >= 100);
            rp3d_test(map2.size() == 0);

            // ----- Copy Constructors ----- //
            Map<int, std::string> map3(map1);
            rp3d_test(map3.capacity() == map1.capacity());
            rp3d_test(map3.size() == map1.size());

            Map<int, int> map4(mAllocator);
            map4.add(Pair<int, int>(1, 10));
            map4.add(Pair<int, int>(2, 20));
            map4.add(Pair<int, int>(3, 30));
            rp3d_test(map4.capacity() >= 3);
            rp3d_test(map4.size() == 3);

            Map<int, int> map5(map4);
            rp3d_test(map5.capacity() == map4.capacity());
            rp3d_test(map5.size() == map4.size());
            rp3d_test(map5[1] == 10);
            rp3d_test(map5[2] == 20);
            rp3d_test(map5[3] == 30);
        }

        void testReserve() {

            Map<int, std::string> map1(mAllocator);
            map1.reserve(15);
            rp3d_test(map1.capacity() >= 15);
            map1.add(Pair<int, std::string>(1, "test1"));
            map1.add(Pair<int, std::string>(2, "test2"));
            rp3d_test(map1.capacity() >= 15);

            map1.reserve(10);
            rp3d_test(map1.capacity() >= 15);

            map1.reserve(100);
            rp3d_test(map1.capacity() >= 100);
            rp3d_test(map1[1] == "test1");
            rp3d_test(map1[2] == "test2");
        }

        void testAddRemoveClear() {

            // ----- Test add() ----- //

            Map<int, int> map1(mAllocator);
            map1.add(Pair<int, int>(1, 10));
            map1.add(Pair<int, int>(8, 80));
            map1.add(Pair<int, int>(13, 130));
            rp3d_test(map1[1] == 10);
            rp3d_test(map1[8] == 80);
            rp3d_test(map1[13] == 130);
            rp3d_test(map1.size() == 3);

            Map<int, int> map2(mAllocator, 15);
            for (int i = 0; i < 1000000; i++) {
                map2.add(Pair<int, int>(i, i * 100));
            }
            bool isValid = true;
            for (int i = 0; i < 1000000; i++) {
                if (map2[i] != i * 100) isValid = false;
            }
            rp3d_test(isValid);

            map1.remove(1);
            map1.add(Pair<int, int>(1, 10));
            rp3d_test(map1.size() == 3);
            rp3d_test(map1[1] == 10);

            map1.add(Pair<int, int>(56, 34));
            rp3d_test(map1[56] == 34);
            rp3d_test(map1.size() == 4);
            map1.add(Pair<int, int>(56, 13), true);
            rp3d_test(map1[56] == 13);
            rp3d_test(map1.size() == 4);

            // ----- Test remove() ----- //

            map1.remove(1);
            rp3d_test(!map1.containsKey(1));
            rp3d_test(map1.containsKey(8));
            rp3d_test(map1.containsKey(13));
            rp3d_test(map1.size() == 3);

            map1.remove(13);
            rp3d_test(map1.containsKey(8));
            rp3d_test(!map1.containsKey(13));
            rp3d_test(map1.size() == 2);

            map1.remove(8);
            rp3d_test(!map1.containsKey(8));
            rp3d_test(map1.size() == 1);

            auto it = map1.remove(56);
            rp3d_test(!map1.containsKey(56));
            rp3d_test(map1.size() == 0);
            rp3d_test(it == map1.end());

            isValid = true;
            for (int i = 0; i < 1000000; i++) {
                map2.remove(i);
            }
            for (int i = 0; i < 1000000; i++) {
                if (map2.containsKey(i)) isValid = false;
            }
            rp3d_test(isValid);
            rp3d_test(map2.size() == 0);

            Map<int, int> map3(mAllocator);
            for (int i=0; i < 1000000; i++) {
                map3.add(Pair<int, int>(i, i * 10));
                map3.remove(i);
            }

            map3.add(Pair<int, int>(1, 10));
            map3.add(Pair<int, int>(2, 20));
            map3.add(Pair<int, int>(3, 30));
            rp3d_test(map3.size() == 3);
            it = map3.begin();
            map3.remove(it++);
            rp3d_test(!map3.containsKey(1));
            rp3d_test(map3.size() == 2);
            rp3d_test(it->second == 20);

            map3.add(Pair<int, int>(56, 32));
            map3.add(Pair<int, int>(23, 89));
            for (it = map3.begin(); it != map3.end();) {
                it = map3.remove(it);
            }
            rp3d_test(map3.size() == 0);

            // ----- Test clear() ----- //

            Map<int, int> map4(mAllocator);
            map4.add(Pair<int, int>(2, 20));
            map4.add(Pair<int, int>(4, 40));
            map4.add(Pair<int, int>(6, 60));
            map4.clear();
            rp3d_test(map4.size() == 0);
            map4.add(Pair<int, int>(2, 20));
            rp3d_test(map4.size() == 1);
            rp3d_test(map4[2] == 20);
            map4.clear();
            rp3d_test(map4.size() == 0);

            Map<int, int> map5(mAllocator);
            map5.clear();
            rp3d_test(map5.size() == 0);

            // ----- Test map with always same hash value for keys ----- //

            Map<TestKey, int> map6(mAllocator);
            for (int i=0; i < 1000; i++) {
                map6.add(Pair<TestKey, int>(TestKey(i), i));
            }
            bool isTestValid = true;
            for (int i=0; i < 1000; i++) {
                if (map6[TestKey(i)] != i) {
                    isTestValid = false;
                }
            }
            rp3d_test(isTestValid);
            for (int i=0; i < 1000; i++) {
                map6.remove(TestKey(i));
            }
            rp3d_test(map6.size() == 0);
        }

        void testContainsKey() {

            Map<int, int> map1(mAllocator);

            rp3d_test(!map1.containsKey(2));
            rp3d_test(!map1.containsKey(4));
            rp3d_test(!map1.containsKey(6));

            map1.add(Pair<int, int>(2, 20));
            map1.add(Pair<int, int>(4, 40));
            map1.add(Pair<int, int>(6, 60));

            rp3d_test(map1.containsKey(2));
            rp3d_test(map1.containsKey(4));
            rp3d_test(map1.containsKey(6));

            map1.remove(4);
            rp3d_test(!map1.containsKey(4));
            rp3d_test(map1.containsKey(2));
            rp3d_test(map1.containsKey(6));

            map1.clear();
            rp3d_test(!map1.containsKey(2));
            rp3d_test(!map1.containsKey(6));
        }

        void testIndexing() {

            Map<int, int> map1(mAllocator);
            map1.add(Pair<int, int>(2, 20));
            map1.add(Pair<int, int>(4, 40));
            map1.add(Pair<int, int>(6, 60));
            rp3d_test(map1[2] == 20);
            rp3d_test(map1[4] == 40);
            rp3d_test(map1[6] == 60);

            map1[2] = 10;
            map1[4] = 20;
            map1[6] = 30;

            rp3d_test(map1[2] == 10);
            rp3d_test(map1[4] == 20);
            rp3d_test(map1[6] == 30);
        }

        void testFind() {

            Map<int, int> map1(mAllocator);
            map1.add(Pair<int, int>(2, 20));
            map1.add(Pair<int, int>(4, 40));
            map1.add(Pair<int, int>(6, 60));
            rp3d_test(map1.find(2)->second == 20);
            rp3d_test(map1.find(4)->second == 40);
            rp3d_test(map1.find(6)->second == 60);
            rp3d_test(map1.find(45) == map1.end());

            map1[2] = 10;
            map1[4] = 20;
            map1[6] = 30;

            rp3d_test(map1.find(2)->second == 10);
            rp3d_test(map1.find(4)->second == 20);
            rp3d_test(map1.find(6)->second == 30);
        }

        void testEquality() {

            Map<std::string, int> map1(mAllocator, 10);
            Map<std::string, int> map2(mAllocator, 2);

            rp3d_test(map1 == map2);

            map1.add(Pair<std::string, int>("a", 1));
            map1.add(Pair<std::string, int>("b", 2));
            map1.add(Pair<std::string, int>("c", 3));

            map2.add(Pair<std::string, int>("a", 1));
            map2.add(Pair<std::string, int>("b", 2));
            map2.add(Pair<std::string, int>("c", 4));

            rp3d_test(map1 == map1);
            rp3d_test(map2 == map2);
            rp3d_test(map1 != map2);

            map2["c"] = 3;

            rp3d_test(map1 == map2);

            Map<std::string, int> map3(mAllocator);
            map3.add(Pair<std::string, int>("a", 1));

            rp3d_test(map1 != map3);
            rp3d_test(map2 != map3);
        }

        void testAssignment() {

           Map<int, int> map1(mAllocator);
           map1.add(Pair<int, int>(1, 3));
           map1.add(Pair<int, int>(2, 6));
           map1.add(Pair<int, int>(10, 30));

           Map<int, int> map2(mAllocator);
           map2 = map1;
           rp3d_test(map2.size() == map1.size());
           rp3d_test(map1 == map2);
           rp3d_test(map2[1] == 3);
           rp3d_test(map2[2] == 6);
           rp3d_test(map2[10] == 30);

           Map<int, int> map3(mAllocator, 100);
           map3 = map1;
           rp3d_test(map3.size() == map1.size());
           rp3d_test(map3 == map1);
           rp3d_test(map3[1] == 3);
           rp3d_test(map3[2] == 6);
           rp3d_test(map3[10] == 30);

           Map<int, int> map4(mAllocator);
           map3 = map4;
           rp3d_test(map3.size() == 0);
           rp3d_test(map3 == map4);

           Map<int, int> map5(mAllocator);
           map5.add(Pair<int, int>(7, 8));
           map5.add(Pair<int, int>(19, 70));
           map1 = map5;
           rp3d_test(map5.size() == map1.size());
           rp3d_test(map5 == map1);
           rp3d_test(map1[7] == 8);
           rp3d_test(map1[19] == 70);
        }

        void testIterators() {

            Map<int, int> map1(mAllocator);

            rp3d_test(map1.begin() == map1.end());

            map1.add(Pair<int, int>(1, 5));
            map1.add(Pair<int, int>(2, 6));
            map1.add(Pair<int, int>(3, 8));
            map1.add(Pair<int, int>(4, -1));

            Map<int, int>::Iterator itBegin = map1.begin();
            Map<int, int>::Iterator it = map1.begin();

            rp3d_test(itBegin == it);

            int size = 0;
            for (auto it = map1.begin(); it != map1.end(); ++it) {
                rp3d_test(map1.containsKey(it->first));
                size++;
            }
            rp3d_test(map1.size() == size);
        }
 };

}

#endif
