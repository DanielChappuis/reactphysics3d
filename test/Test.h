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

#ifndef TEST_H
#define TEST_H

// Libraries
#include <string>
#include <iostream>
#include <cassert>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Macros
#define rp3d_test(condition) applyTest(condition, #condition, __FILE__, __LINE__);
#define rp3d_fail(text) applyFail(text, __FILE__, __LINE__);

// Class Test
/**
 * This abstract class represents a unit test. To create a unit test, you simply
 * need to create a class that inherits from the Test class, override the run() method and
 * use the test() and fail() macros.
 */
class Test {

    private :

        // ---------- Attributes ---------- //

        /// Name of the test
        std::string mName;

        /// Number of tests that passed
        long mNbPassedTests;

        /// Number of tests that failed
        long mNbFailedTests;

        /// Output stream
        std::ostream* mOutputStream;

        // ---------- Methods ---------- //

        /// Copy constructor is private
        Test(const Test&);

        /// Assignment operator is private
        Test& operator=(const Test& test);

    protected :

        // ---------- Methods ---------- //

        /// Called to test a boolean condition.
        /// This method should not be called directly in your test but you should
        /// call test() instead (macro)
        void applyTest(bool condition, const std::string& testText,
                       const char* filename, long lineNumber);

        /// Called when a test has failed.
        /// This method should not be called directly in your test buy you should
        /// call fail() instead (macro)
        void applyFail(const std::string& testText, const char* filename, long lineNumber);

    public :

        // ---------- Methods ---------- //

        /// Constructor
        Test(const std::string& name, std::ostream* stream = &std::cout);

        /// Destructor
        virtual ~Test();

        /// Return the number of passed tests
        long getNbPassedTests() const;

        /// Return the number of failed tests
        long getNbFailedTests() const;

        /// Return the output stream
        const std::ostream* getOutputStream() const;

        /// Set the output stream
        void setOutputStream(std::ostream *stream);

        /// Run the unit test
        virtual void run() = 0;

        /// Called when a test passed
        void succeed();

        /// Reset the unit test
        virtual void reset();

        /// Display the report of the unit test and return the number of failed tests
        long report() const;
};

// Called when a test passed
inline void Test::succeed() {
    mNbPassedTests++;
}

// Reset the unit test
inline void Test::reset() {
    mNbPassedTests = 0;
    mNbFailedTests = 0;
}

// Return the number of passed tests
inline long Test::getNbPassedTests() const {
    return mNbPassedTests;
}

// Return the number of failed tests
inline long Test::getNbFailedTests() const {
    return mNbFailedTests;
}

// Return the output stream
inline const std::ostream* Test::getOutputStream() const {
    return mOutputStream;
}

// Set the output stream
inline void Test::setOutputStream(std::ostream* stream) {
    mOutputStream = stream;
}

}

#endif
