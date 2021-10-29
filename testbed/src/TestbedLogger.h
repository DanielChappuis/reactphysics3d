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

#ifndef TESTBED_LOGGER_H
#define	TESTBED_LOGGER_H

// Libraries
#include <reactphysics3d/utils/DefaultLogger.h>
#include <unordered_map>
#include <string>

/// Class TestbedApplication
class TestbedLogger : public reactphysics3d::Logger {

    private:

        /// Map a log format to the given formatter object
        std::unordered_map<reactphysics3d::DefaultLogger::Format, reactphysics3d::DefaultLogger::Formatter*> mFormatters;

        /// Map the name of a world with the corresponding log destination
        std::unordered_map<std::string, reactphysics3d::DefaultLogger::Destination*> mMapWorldToDestinations;

        reactphysics3d::DefaultLogger::StreamDestination* mStandardOutputDestination;

        /// Mutex
        std::mutex mMutex;

        // -------------------- Methods -------------------- //

        /// Return the corresponding formatter
        reactphysics3d::DefaultLogger::Formatter* getFormatter(reactphysics3d::DefaultLogger::Format format) const;

        /// Add a stream destination to the logger
        void addStreamDestination(std::ostream& outputStream, reactphysics3d::uint logLevelFlag, reactphysics3d::DefaultLogger::Format format);

    public:

        /// Constructor
        TestbedLogger();

        /// Destructor
        ~TestbedLogger();

        /// Add a log file destination to the logger
        void addFileDestination(const std::string& worldName, reactphysics3d::uint logLevelFlag, reactphysics3d::DefaultLogger::Format format);


        /// Log something
        virtual void log(Level level, const std::string& physicsWorldName, Category category, const std::string& message, const char* filename, int lineNumber) override;
};

#endif
