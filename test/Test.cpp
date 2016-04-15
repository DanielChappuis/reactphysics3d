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

// Libraries
#include "Test.h"
#include <typeinfo>
#include <iomanip>

using namespace reactphysics3d;

/// Constructor
Test::Test(const std::string& name, std::ostream* stream)
     : mName(name), mNbPassedTests(0), mNbFailedTests(0), mOutputStream(stream) {

}

/// Destructor
Test::~Test() {

}

// Called to test a boolean condition.
// This method should not be called directly in your test but you should call test() instead (macro)
void Test::applyTest(bool condition, const std::string& testText,
                     const char* filename, long lineNumber) {

    // If the boolean condition is true
    if (condition) {

        // The test passed, call the succeed() method
        succeed();
    }
    else {  // If the boolean condition is false

        // The test failed, call the applyFail() method
        applyFail(testText, filename, lineNumber);
    }
}

// Called when a test has failed.
// This method should not be called directly in your test buy you should call fail() instead (macro)
void Test::applyFail(const std::string& testText, const char* filename, long lineNumber) {

    if (mOutputStream) {

        // Display the failure message
        *mOutputStream << mName << " failure : (" << testText << "), " <<
                  filename << "(line " << lineNumber << ")" << std::endl;
    }

    // Increase the number of failed tests
    mNbFailedTests++;
}

/// Display the report of the unit test and return the number of failed tests
long Test::report() const {

    if(mOutputStream) {
        *mOutputStream << std::left << std::setw(30) << std::setfill(' ')
                       << "Test " + mName + " :" << std::setw(10) << "Passed:  "
                       << std::setw(8) << mNbPassedTests
                       << std::setw(13) << "Failed: "
                       << std::setw(8) << mNbFailedTests  << std::endl;
      }

    // Return the number of failed tests
    return mNbFailedTests;
}
