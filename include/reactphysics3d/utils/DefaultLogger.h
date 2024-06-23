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

#ifndef REACTPHYSICS3D_DEFAULT_LOGGER_H
#define REACTPHYSICS3D_DEFAULT_LOGGER_H

// Libraries
#include <reactphysics3d/utils/Logger.h>
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/containers/Map.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <ctime>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Logger
/**
 * This class is the default logger class used to log information, warnings
 * or errors during the execution of the library code for easier debugging.
 */
class DefaultLogger : public Logger {

    public:

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
                virtual std::string format(const time_t& time, const std::string& physicsWorldName, const std::string& message, Level level, Category category,
                                           const char* filename, int lineNumber) = 0;

                /// Return the current date and time
                std::tm getLocalTime(const std::time_t& time) const {

                    std::tm bt = std::tm();

                    // This is because std::localtime is not thread-safe

#if defined(__unix__)
                    localtime_r(&time, &bt);
#elif defined(_MSC_VER)
                    localtime_s(&bt, &time);
#else
                    static std::mutex mtx;
                    std::lock_guard<std::mutex> lock(mtx);
                    bt = *std::localtime(&time);
#endif

                    return bt;
                }

        };

        // Class TextFormatter
        /**
         * Format the logs with simple text
         */
        class TextFormatter : public Formatter {

            public:

                /// Constructor
                TextFormatter() {

                }

                /// Destructor
                virtual ~TextFormatter() override {

                }

                /// Return the header to write at the beginning of the stream
                virtual std::string getHeader() const override {

                    // Get current date
                    auto now = std::chrono::system_clock::now();
                    std::time_t time = std::chrono::system_clock::to_time_t(now);

                    auto localTime = getLocalTime(time);

                    std::stringstream ss;
                    ss << "ReactPhysics3D Logs" << std::endl;
                    ss << "ReactPhysics3D Version: " << RP3D_VERSION << std::endl;
                    ss << "Date: " << std::put_time(&localTime, "%Y-%m-%d") << std::endl;
                    ss << "---------------------------------------------------------" << std::endl;

                    return ss.str();
                }

                /// Format a log message
                virtual std::string format(const time_t& time, const std::string& physicsWorldName, const std::string& message,
                                           Level level, Category category, const char* filename, int lineNumber) override {
                    std::stringstream ss;

                    auto localTime = getLocalTime(time);

                    // Time
                    ss << std::put_time(&localTime, "%X") << " ";

                    // World
                    ss << "World:" << physicsWorldName << std::endl;

                    // Level
                    ss << getLevelName(level) << " ";

                    // Category
                    ss << getCategoryName(category) << " ";

                    // Message
                    ss << message << std::endl;

                    // Filename
                    ss << " (in file " << std::string(filename);

                    // Line number
                    ss << " at line " << std::to_string(lineNumber) << ")";

                    return ss.str();
                }
        };

        // Class HtmlFormatter
        /**
         * Format the logs with HTML
         */
        class HtmlFormatter : public Formatter {

            private:

                /// Return the header to write at the beginning of the stream
                virtual std::string getHeader() const override {

                    // Get current date
                    auto now = std::chrono::system_clock::now();
                    auto time = std::chrono::system_clock::to_time_t(now);
                    auto localTime = getLocalTime(time);

                    std::stringstream ss;
                    ss << "<!DOCTYPE HTML>" << std::endl;
                    ss << "<html>" << std::endl;
                    ss << "<head>" << std::endl;
                    ss << "<title>ReactPhysics3D Logs</title>" << std::endl;
                    ss << "<style>" << generateCSS() << "</style>" << std::endl;
                    ss << "</head>" << std::endl;
                    ss << "<body>" << std::endl;
                    ss << "<h1>ReactPhysics3D Logs</h1>" << std::endl;
                    ss << "<div class='general_info'>" << std::endl;
                    ss << "<p>ReactPhysics3D version: " << RP3D_VERSION << "</p>" << std::endl;
                    ss << "<p>Date: " << std::put_time(&localTime, "%Y-%m-%d") << "</p>" << std::endl;
                    ss << "</div>" << std::endl;
                    ss << "<hr>";

                    return ss.str();
                }

                /// Return the tail to write at the end of the stream
                virtual std::string getTail() const override {

                    std::stringstream ss;

                    ss << "</body>" << std::endl;
                    ss << "</html>" << std::endl;

                    return ss.str();
                }

            std::string generateCSS() const {
                return "body {"
                       "  background-color: #e6e6e6;"
                       "  font-family: SFMono-Regular,Menlo,Monaco,Consolas,'Liberation Mono','Courier New',monospace; "
                       "} "
                      "body > div { clear:both; } "
                      "body > div > div { float: left; } "
                      "h1 {"
                      "  margin: 5px 5px 5px 5px;"
                      "} "
                      ".general_info > p {"
                        "margin: 5px;"
                        "font-weight: bold;"
                      "} "
                      ".line { "
                        "font-size: 13px; "
                        "color: #212529; "
                        "margin: 5px 5px 5px 5px; "
                        "padding: 5px 0px 5px 0px; "
                      "} "
                      ".time { "
                         "margin-right: 20px; "
                         "width: 5%; "
                      "} "
                      ".world-name { "
                         "margin-right: 20px; "
                         "width: 5%; "
                      "}"
                      ".level { "
                         "margin-right: 20px; "
                         "width: 10%; "
                      "}"
                      ".category { "
                         "margin-right: 20px; "
                         "width: 10%; "
                         "font-weight: bold; "
                      "}"
                      ".message { "
                         "margin-right: 20px; "
                        "color: #2e2e2e; "
                        "word-wrap: break-word; "
                        "width: 40%; "
                      "} "
                      ".file { "
                         "margin-right: 20px; "
                         "word-wrap: break-word; "
                         "width: 20%; "
                      "}"
                      ".body > .category, .body > .message { "
                        "color: #00994d;"
                      "} "
                      ".world > .category, .world > .message { "
                        "color: #3477DB; "
                      "} "
                      ".joint .category, .joint > .message { "
                        "color: #bf8040; "
                      "} "
                      ".collider .category, .collider > .message { "
                        "color: #9933ff; "
                      "} "
                      ".warning { "
                        "color: #ff9900 !important; "
                      "} "
                      ".error { "
                        "color: red !important; "
                      "} ";
            }

            /// Convert a string to lower case
            std::string toLowerCase(const std::string& text) {
                std::string textLower = text;
                std::transform(textLower.begin(), textLower.end(), textLower.begin(), ::tolower);
                return textLower;
            }

            public:

                /// Constructor
                HtmlFormatter() {

                }

                /// Destructor
                virtual ~HtmlFormatter() override {

                }

                /// Format a log message
                virtual std::string format(const time_t& time, const std::string& physicsWorldName, const std::string& message, Level level,
                                           Category category, const char* filename, int lineNumber) override {

                    std::stringstream ss;

                    auto localTime = getLocalTime(time);

                    ss << "<div class='line " + toLowerCase(getCategoryName(category)) + " " + toLowerCase(getLevelName(level)) + "'>";

                    // Time
                    ss << "<div class='time'>";
                    ss << std::put_time(&localTime, "%X");
                    ss << "</div>";

                    // Message
                    ss << "<div class='world-name'>";
                    ss << physicsWorldName;
                    ss << "</div>";

                    // Level
                    ss << "<div class='level'>";
                    ss << getLevelName(level);
                    ss << "</div>";

                    // Category
                    ss << "<div class='category'>";
                    ss << getCategoryName(category);
                    ss << "</div>";

                    // Message
                    ss << "<div class='message " << toLowerCase(getCategoryName(category)) <<
                          " " + toLowerCase(getLevelName(level)) << "'>" << std::endl;
                    ss << message;
                    ss << "</div>";

                    // Filename + line number
                    ss << "<div class='file'> (in file " << std::string(filename) <<
                          " at line " << std::to_string(lineNumber) << ")" << std::endl;
                    ss << "</div>";

                    ss << "</div>";

                    return ss.str();
                }
        };


        // Class Destination
        /**
         * destination for the logs
         */
        class Destination {

            public:

                /// Maximum Log level flag for this destination
                uint maxLevelFlag;

                /// Pointer to the formatter
                Formatter* formatter;

                /// Constructor
                Destination(uint maxLevelFlag, Formatter* logFormatter)
                    : maxLevelFlag(maxLevelFlag), formatter(logFormatter) {

                }

                /// Destructor
                virtual ~Destination() {

                }

                /// Write a message into the output stream
                virtual void write(const time_t& time, const std::string& physicsWorldName, const std::string& message, Level level, Category category, const char* filename, int lineNumber) = 0;

                /// Return the size in bytes of the type
                virtual size_t getSizeBytes() const=0;
        };

        // Class FileDestination
        /**
         * File destination for the logs
         */
        class FileDestination : public Destination {

            private:

                std::string mFilePath;

                /// Output file stream
                std::ofstream mFileStream;

            public:

                /// Constructor
                FileDestination(const std::string& filePath, uint maxLevelFlag, Formatter* formatter)
                   :Destination(maxLevelFlag, formatter), mFilePath(filePath),
                    mFileStream(filePath, std::ios::binary) {

                    assert(mFileStream.is_open());

                    // Write the header
                    mFileStream << formatter->getHeader() << std::endl;
                }

                /// Destructor
                virtual ~FileDestination() override {

                    // Write the tail
                    mFileStream << formatter->getTail() << std::endl;

                    if (mFileStream.is_open()) {

                        // Close the stream
                        mFileStream.close();
                    }
                }

                /// Write a message into the output stream
                virtual void write(const time_t& time, const std::string& physicsWorldName, const std::string& message, Level level, Category category,
                                   const char* filename, int lineNumber) override {

                    if (static_cast<int>(level) <= static_cast<int>(maxLevelFlag)) {
                        mFileStream << formatter->format(time, physicsWorldName, message, level, category, filename, lineNumber) << std::endl;
                    }
                }

                /// Return the size in bytes of the type
                virtual size_t getSizeBytes() const override {
                    return sizeof(FileDestination);
                }
        };

        // Class TextFormatter
        /**
         * Stream destination for the logs
         */
        class StreamDestination : public Destination {

            private:

                /// Output stream
                std::ostream& mOutputStream;

            public:

                /// Constructor
                StreamDestination(std::ostream& outputStream, uint maxlevelFlag, Formatter* formatter)
                   :Destination(maxlevelFlag, formatter), mOutputStream(outputStream) {

                    // Writer the head
                    mOutputStream << formatter->getHeader() << std::endl;
                }

                /// Destructor
                virtual ~StreamDestination() override {

                    // Writer the tail
                    mOutputStream << formatter->getTail() << std::endl;
                }

                /// Write a message into the output stream
                virtual void write(const time_t& time, const std::string& physicsWorldName, const std::string& message, Level level, Category category,
                                   const char* filename, int lineNumber) override {

                    if (static_cast<int>(level) <= static_cast<int>(maxLevelFlag)) {
                        mOutputStream << formatter->format(time, physicsWorldName, message, level, category, filename, lineNumber) << std::endl << std::flush;
                    }
                }

                /// Return the size in bytes of the type
                virtual size_t getSizeBytes() const override {
                    return sizeof(StreamDestination);
                }
        };


    protected:

        // -------------------- Attributes -------------------- //

        /// Memory allocator
        MemoryAllocator& mAllocator;

        /// All the log destinations
        Array<Destination*> mDestinations;

        /// Map a log format to the given formatter object
        Map<Format, Formatter*> mFormatters;

        /// Mutex
        std::mutex mMutex;

        // -------------------- Methods -------------------- //

        /// Return the corresponding formatter
        Formatter* getFormatter(Format format) const;

        /// Constructor
        DefaultLogger(MemoryAllocator& allocator);

        /// Destructor
        virtual ~DefaultLogger() override;

    public :

        // -------------------- Methods -------------------- //

        /// Add a file destination to the logger
        void addFileDestination(const std::string& filePath, uint logLevelFlag, Format format);

        /// Add a stream destination to the logger
        void addStreamDestination(std::ostream& outputStream, uint logLevelFlag, Format format);

        /// Remove all logs destination previously set
        void removeAllDestinations();

        /// Log something
        virtual void log(Level level, const std::string& physicsWorldName, Category category, const std::string& message, const char* filename, int lineNumber) override;

        // ---------- Friendship ---------- //

        friend class PhysicsCommon;
};

}

namespace std {

  // Hash function for struct VerticesPair
  template<> struct hash<reactphysics3d::DefaultLogger::Format> {

    size_t operator()(const reactphysics3d::DefaultLogger::Format format) const {

        return static_cast<size_t>(format);
    }
  };
}

#endif
