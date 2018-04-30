/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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

#ifdef IS_LOGGING_ACTIVE

// Libraries
#include "Logger.h"
#include "memory/MemoryManager.h"

using namespace reactphysics3d;

// Constructor
Logger::Logger()
       : mDestinations(MemoryManager::getBaseAllocator()), mFormatters(MemoryManager::getBaseAllocator())
{

    // Create the log formatters
    mFormatters.add(Pair<Format, Formatter*>(Format::Text, new TextFormatter()));
    mFormatters.add(Pair<Format, Formatter*>(Format::HTML, new HtmlFormatter()));
}

// Destructor
Logger::~Logger() {

    removeAllDestinations();

    // Remove all the loggers
    for (auto it = mFormatters.begin(); it != mFormatters.end(); ++it) {

       delete it->second;
    }
}

// Return the corresponding formatter
Logger::Formatter* Logger::getFormatter(Format format) const {

   auto it = mFormatters.find(format);
   if (it != mFormatters.end()) {
       return it->second;
   }

   return nullptr;
}

// Add a log file destination to the logger
void Logger::addFileDestination(const std::string& filePath, uint logLevelFlag, Format format) {

    FileDestination* destination = new FileDestination(filePath, logLevelFlag, getFormatter(format));
    mDestinations.add(destination);
}

/// Add a stream destination to the logger
void Logger::addStreamDestination(std::ostream& outputStream, uint logLevelFlag, Format format) {

    StreamDestination* destination = new StreamDestination(outputStream, logLevelFlag, getFormatter(format));
    mDestinations.add(destination);
}

// Remove all logs destination previously set
void Logger::removeAllDestinations() {

    // Delete all the destinations
    for (uint i=0; i<mDestinations.size(); i++) {
        delete mDestinations[i];
    }

    mDestinations.clear();
}

// Log something
void Logger::log(Level level, Category category, const std::string& message) {

    // Get current time
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);

    mMutex.lock();

    // For each destination
    for (auto it = mDestinations.begin(); it != mDestinations.end(); ++it) {

        (*it)->write(time, message, level, category);
    }

    mMutex.unlock();
}

#endif
