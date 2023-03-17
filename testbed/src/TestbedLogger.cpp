/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2021 Daniel Chappuis                                       *
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
#include "TestbedLogger.h"

using namespace reactphysics3d;

// Constructor
TestbedLogger::TestbedLogger() {

    // Create the log formatters
    mFormatters.insert({reactphysics3d::DefaultLogger::Format::Text, new reactphysics3d::DefaultLogger::TextFormatter()});
    mFormatters.insert({reactphysics3d::DefaultLogger::Format::HTML, new reactphysics3d::DefaultLogger::HtmlFormatter()});

    // Add destination to send warning and errors to standard output
    uint warningsErrors = static_cast<uint>(reactphysics3d::Logger::Level::Warning);
    addStreamDestination(std::cout, warningsErrors, reactphysics3d::DefaultLogger::Format::Text);
}

// Destructor
TestbedLogger::~TestbedLogger() {

    // Delete all the destinations
    for (auto& item: mMapWorldToDestinations) {
       delete item.second;
    }

    delete mStandardOutputDestination;
}

// Log something
void TestbedLogger::log(Level level, const std::string& physicsWorldName, Category category, const std::string& message, const char* filename, int lineNumber) {

    // Get current time
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);

    // Get the destination file for this world
    DefaultLogger::Destination* destination = mMapWorldToDestinations[physicsWorldName];

    if (destination != nullptr) {

        mMutex.lock();

        // Write the log file into the file of the corresponding scene
        destination->write(time, physicsWorldName, message, level, category, filename, lineNumber);

        // Write the log into the standard output
        mStandardOutputDestination->write(time, physicsWorldName, message, level, category, filename, lineNumber);

        mMutex.unlock();
    }
}

// Return the corresponding format
DefaultLogger::Formatter* TestbedLogger::getFormatter(DefaultLogger::Format format) const {

   auto it = mFormatters.find(format);
   if (it != mFormatters.end()) {
       return it->second;
   }

   return nullptr;
}

// Add a log file destination to the logger
void TestbedLogger::addFileDestination(const std::string& worldName, reactphysics3d::uint logLevelFlag, reactphysics3d::DefaultLogger::Format format) {

    std::string filePath = "rp3d_log_" + worldName + ".html";
    reactphysics3d::DefaultLogger::FileDestination* destination = new reactphysics3d::DefaultLogger::FileDestination(filePath, logLevelFlag, getFormatter(format));
    mMapWorldToDestinations.insert({worldName, destination});
}

// Add a stream destination to the logger
void TestbedLogger::addStreamDestination(std::ostream& outputStream, reactphysics3d::uint logLevelFlag, reactphysics3d::DefaultLogger::Format format) {

    mStandardOutputDestination = new reactphysics3d::DefaultLogger::StreamDestination(outputStream, logLevelFlag, getFormatter(format));
}
