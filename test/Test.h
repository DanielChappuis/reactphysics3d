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

#ifndef TEST_H
#define TEST_H

// Libraries
#include <string>
#include <iostream>
#include <cassert>

/// Reactphysics3D namespace
namespace reactphysics3d {

/**
 * This abstract class represents a unit test
 */
class Test {

    private :

        // ---------- Attributes ---------- //

        /// Number of tests that passed
        uint mNbPassedTests;

        /// Number of tests that failed
        uint mNbFailedTests;

        /// Output stream
        std::ostream* mOutputStream;

        // ---------- Methods ---------- //

        /// Copy constructor is private
        Test(const Test&);

        /// Assignment operator is private
        Test& operator=(const Test& test);

    protected :

    public :

        // ---------- Methods ---------- //

        /// Constructor
        Test(std::ostream* stream = &std::cout);

        /// Destructor
        ~Test();

        /// Return the number of passed tests
        uint getNbPassedTests() const;

        /// Return the number of failed tests
        uint getNbFailedTests() const;

        /// Return the output stream
        const std::ostream* getOutputStream() const;

        /// Set the output stream
        void setOutputStream(const std::ostream* stream);

        /// Run the unit test
        virtual void run() = 0;

        /// Reset the unit test
        virtual void reset();

        /// Display the report of the unit test and return the number of failed tests
        uint report() const;
};

/// Reset the unit test
inline void Test::reset() {
    mNbPassedTests = 0;
    mNbFailedTests = 0;
}

/// Return the number of passed tests
inline uint Test::getNbPassedTests() const {
    return mNbPassedTests;
}

/// Return the number of failed tests
inline uint Test::getNbFailedTests() const {
    return mNbFailedTests;
}

/// Return the output stream
inline const std::ostream* Test::getOutputStream() const {
    return mOutputStream;
}

/// Set the output stream
inline void Test::setOutputStream(const std::ostream* stream) {
    mOutputStream = stream;
}

}

#endif
