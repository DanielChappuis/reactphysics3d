/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/utils/DefaultLogger.h>
#include <reactphysics3d/memory/MemoryManager.h>

using namespace reactphysics3d;

// Constructor
DefaultLogger::DefaultLogger(MemoryAllocator& allocator)
       : mAllocator(allocator), mDestinations(allocator), mFormatters(allocator)
{
    // Create the log formatters
    mFormatters.add(Pair<Format, Formatter*>(Format::Text, new TextFormatter()));
    mFormatters.add(Pair<Format, Formatter*>(Format::HTML, new HtmlFormatter()));
}

// Destructor
DefaultLogger::~DefaultLogger() {

    removeAllDestinations();

    // Remove all the formatters
    for (auto it = mFormatters.begin(); it != mFormatters.end(); ++it) {

       delete it->second;
    }
}

// Return the corresponding formatter
DefaultLogger::Formatter* DefaultLogger::getFormatter(Format format) const {

   auto it = mFormatters.find(format);
   if (it != mFormatters.end()) {
       return it->second;
   }

   return nullptr;
}

// Add a log file destination to the logger
void DefaultLogger::addFileDestination(const std::string& filePath, uint logLevelFlag, Format format) {

    // Make sure capacity is an integral multiple of alignment
    const size_t allocatedSize = std::ceil(sizeof(FileDestination) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

    FileDestination* destination = new (mAllocator.allocate(allocatedSize)) FileDestination(filePath, logLevelFlag, getFormatter(format));
    mDestinations.add(destination);
}

/// Add a stream destination to the logger
void DefaultLogger::addStreamDestination(std::ostream& outputStream, uint logLevelFlag, Format format) {

    // Make sure capacity is an integral multiple of alignment
    const size_t allocatedSize = std::ceil(sizeof(StreamDestination) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

    StreamDestination* destination = new (mAllocator.allocate(allocatedSize)) StreamDestination(outputStream, logLevelFlag, getFormatter(format));
    mDestinations.add(destination);
}

// Remove all logs destination previously set
void DefaultLogger::removeAllDestinations() {

    // Delete all the destinations
    for (uint32 i=0; i<mDestinations.size(); i++) {

        size_t size = mDestinations[i]->getSizeBytes();

        mDestinations[i]->~Destination();

        // Make sure capacity is an integral multiple of alignment
        size = std::ceil(size / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

        mAllocator.release(mDestinations[i], size);
    }

    mDestinations.clear();
}

// Log something
void DefaultLogger::log(Level level, const std::string& physicsWorldName, Category category, const std::string& message, const char* filename, int lineNumber) {

    // Get current time
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);

    mMutex.lock();

    // For each destination
    for (auto it = mDestinations.begin(); it != mDestinations.end(); ++it) {

        (*it)->write(time, physicsWorldName, message, level, category, filename, lineNumber);
    }

    mMutex.unlock();
}
