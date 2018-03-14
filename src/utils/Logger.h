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

#ifndef REACTPHYSICS3D_LOGGER_H
#define REACTPHYSICS3D_LOGGER_H

#ifdef IS_LOGGING_ACTIVE

// Libraries
#include "containers/List.h"
#include "containers/Map.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Logger
/**
 * This class is used to log information, warnings or errors during the execution of the
 * library code for easier debugging.
 */
class Logger {

    public:

        /// Log verbosity levels
        enum class Level {Error = 1, Warning = 2, Info = 4};

        /// Log categories
        enum class Category {World, Body, Joint};

        /// Log verbosity level
        enum class Format {Text, HTML};

        /// Log formatter
        class Formatter {

            public:

                /// Constructor
                Formatter() {

                }

                /// Destructor
                virtual ~Formatter() {

                }

                /// Return the header to write at the beginning of the stream
                virtual std::string getHeader() const {
                    return "";
                }

                /// Return the tail to write at the end of the stream
                virtual std::string getTail() const {
                    return "";
                }

                /// Format a log message
                virtual std::string format(const std::string& message, Level level, Category category) = 0;
        };

        class TextFormatter : public Formatter {

            public:

                /// Constructor
                TextFormatter() {

                }

                /// Destructor
                virtual ~TextFormatter() {

                }

                /// Format a log message
                virtual std::string format(const std::string& message, Level level,
                                           Category category) override {
                    return message;
                }
        };

        class HtmlFormatter : public Formatter {

            private:

                /// Return the header to write at the beginning of the stream
                virtual std::string getHeader() const override {

                    std::stringstream ss;

                    ss << "<!DOCTYPE HTML>" << std::endl;
                    ss << "<html>" << std::endl;
                    ss << "<head>" << std::endl;
                    ss << "<title>ReactPhysics3D Logs</title>" << std::endl;
                    ss << "</head>" << std::endl;
                    ss << "<body>" << std::endl;

                    return ss.str();
                }

                /// Return the tail to write at the end of the stream
                virtual std::string getTail() const override {

                    std::stringstream ss;

                    ss << "</body>" << std::endl;
                    ss << "</html>" << std::endl;

                    return ss.str();
                }

            public:

                /// Constructor
                HtmlFormatter() {

                }

                /// Destructor
                virtual ~HtmlFormatter() {

                }

                /// Format a log message
                virtual std::string format(const std::string& message, Level level,
                                           Category category) override {

                    std::stringstream ss;

                    ss << "<p>";
                    ss << message;
                    ss << "</p>";

                    return ss.str();
                }
        };


        /// Log destination
        class Destination {

            public:

                /// Log level flag for this destination
                uint levelFlag;

                /// Pointer to the formatter
                Formatter* formatter;

                /// Constructor
                Destination(uint levelFlag, Formatter* logFormatter)
                    : levelFlag(levelFlag), formatter(logFormatter) {

                }

                /// Destructor
                virtual ~Destination() {

                }

                /// Write a message into the output stream
                virtual void write(const std::string& message, Level level, Category category) = 0;
        };

        class FileDestination : public Destination {

            private:

                std::string mFilePath;

                /// Output file stream
                std::ofstream mFileStream;

            public:

                /// Constructor
                FileDestination(const std::string& filePath, uint levelFlag, Formatter* formatter)
                   :Destination(levelFlag, formatter), mFilePath(filePath),
                    mFileStream(filePath, std::ios::binary) {

                    if(!mFileStream.is_open()) {
                        throw(std::runtime_error("ReactPhysics3D Logger: Unable to open an output stream to file " + mFilePath));
                    }

                    // Writer the head
                    mFileStream << formatter->getHeader() << std::endl;
                }

                /// Destructor
                virtual ~FileDestination() override {

                    // Writer the tail
                    mFileStream << formatter->getTail() << std::endl;

                    if (mFileStream.is_open()) {

                        // Close the stream
                        mFileStream.close();
                    }
                }

                /// Write a message into the output stream
                virtual void write(const std::string& message, Level level, Category category) override {
                    mFileStream << formatter->format(message, level, category) << std::endl;
                }
        };

        /// Stream destination to output the logs into a stream
        class StreamDestination : public Destination {

            private:

                /// Output stream
                std::ostream& mOutputStream;

            public:

                /// Constructor
                StreamDestination(std::ostream& outputStream, uint levelFlag, Formatter* formatter)
                   :Destination(levelFlag, formatter), mOutputStream(outputStream) {

                    // Writer the head
                    mOutputStream << formatter->getHeader() << std::endl;
                }

                /// Destructor
                virtual ~StreamDestination() override {

                    // Writer the tail
                    mOutputStream << formatter->getTail() << std::endl;
                }

                /// Write a message into the output stream
                virtual void write(const std::string& message, Level level, Category category) override {
                    mOutputStream << message << std::endl;
                }
        };


    private:

        // -------------------- Attributes -------------------- //

        /// All the log destinations
        List<Destination*> mDestinations;

        /// Map a log format to the given formatter object
        Map<Format, Formatter*> mFormatters;

        // -------------------- Methods -------------------- //

        /// Return the corresponding formatter
        Formatter* getFormatter(Format format) const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Logger();

        /// Destructor
        ~Logger();

        /// Add a file destination to the logger
        void addFileDestination(const std::string& filePath, uint logLevelFlag, Format format);

        /// Add a stream destination to the logger
        void addStreamDestination(std::ostream& outputStream, uint logLevelFlag, Format format);

        /// Remove all logs destination previously set
        void removeAllDestinations();

        /// Log something
        void log(Level level, Category category, const std::string& message);
};


// Use this macro to log something
#define RP3D_LOG(logger, level, category, message) logger.log(level, category, message)


}

#else   // If logger is not active

// Empty macro in case logs are not enabled
#define RP3D_LOG(logger, level, message)

#endif

// Hash function for struct VerticesPair
namespace std {

  template<> struct hash<reactphysics3d::Logger::Format> {

    size_t operator()(const reactphysics3d::Logger::Format format) const {

        return static_cast<size_t>(format);
    }
  };
}

#endif
