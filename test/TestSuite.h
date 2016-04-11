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

#ifndef TEST_SUITE_H
#define TEST_SUITE_H

// Libraries
#include "Test.h"
#include <vector>
#include <stdexcept>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestSuite
/**
 * This class represents a test suite that can
 * contains multiple unit tests. You can also add a test suite inside
 * another test suite (all the tests of the first test suite will be added
 * to the second one).
 */
class TestSuite {

    private :

        // ---------- Attributes ---------- //

        /// Name of the test suite
        std::string mName;

        /// Output stream
        std::ostream* mOutputStream;

        /// All the tests of the test suite
        std::vector<Test*> mTests;

        // ---------- Methods ---------- //

        /// Reset the test suite
        void reset();

        /// Private copy-constructor
        TestSuite(const TestSuite& testSuite);

        /// Private assigmnent operator
        TestSuite& operator=(const TestSuite testSuite);

  public :

        // ---------- Methods ---------- //

        /// Constructor
        TestSuite(const std::string& name, std::ostream* outputStream = &std::cout);

        /// Return the name of the test suite
        std::string getName() const;

        /// Return the number of passed tests
        long getNbPassedTests() const;

        /// Return the number of failed tests
        long getNbFailedTests() const;

        /// Return the output stream
        const std::ostream* getOutputStream() const;

        /// Set the output stream
        void setOutputStream(std::ostream* outputStream);

        /// Add a unit test in the test suite
        void addTest(Test* test);

        /// Add a test suite to the current test suite
        void addTestSuite(const TestSuite& testSuite);

        /// Launch the tests of the test suite
        void run();

        /// Display the tests report and return the number of failed tests
        long report() const;

        // Delete all the tests
        void clear();

};

// Return the name of the test suite
inline std::string TestSuite::getName() const {
    return mName;
}

// Return the output stream
inline const std::ostream* TestSuite::getOutputStream() const {
    return mOutputStream;
}

// Set the output stream
inline void TestSuite::setOutputStream(std::ostream* outputStream) {
    mOutputStream = outputStream;
}

}

#endif
