/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

// If logging is enabled
#ifdef IS_LOGGING_ACTIVE

// Libraries
#include "containers/List.h"
#include "containers/Map.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <mutex>

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
        enum class Level {Error = 1, Warning = 2, Information = 4};

        /// Log categories
        enum class Category {World, Body, Joint, ProxyShape};

        /// Log verbosity level
        enum class Format {Text, HTML};

        /// Return the name of a category
        static std::string getCategoryName(Category category) {

            switch(category) {
                case Category::World: return "World";
                case Category::Body: return "Body";
                case Category::Joint: return "Joint";
                case Category::ProxyShape: return "ProxyShape";
            }

            assert(false);
            return "";
        }

        /// Return the name of a level
        static std::string getLevelName(Level level) {

            switch(level) {
                case Level::Information: return "Information";
                case Level::Warning: return "Warning";
                case Level::Error: return "Error";
            }

            assert(false);
            return "";
        }

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
                virtual std::string format(const time_t& time, const std::string& message, Level level, Category category) = 0;
        };

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
                    auto time = std::chrono::system_clock::to_time_t(now);

                    std::stringstream ss;
                    ss << "ReactPhysics3D Logs" << std::endl;
                    ss << "ReactPhysics3D Version: " << RP3D_VERSION << std::endl;
                    ss << "Date: " << std::put_time(std::localtime(&time), "%Y-%m-%d") << std::endl;
                    ss << "---------------------------------------------------------" << std::endl;

                    return ss.str();
                }

                /// Format a log message
                virtual std::string format(const time_t& time, const std::string& message,
                                           Level level, Category category) override {
                    std::stringstream ss;

                    // Time
                    ss << std::put_time(std::localtime(&time), "%X") << " ";

                    // Level
                    ss << getLevelName(level) << " ";

                    // Category
                    ss << getCategoryName(category) << " ";

                    // Message
                    ss << message << std::endl;

                    return ss.str();
                }
        };

        class HtmlFormatter : public Formatter {

            private:

                /// Return the header to write at the beginning of the stream
                virtual std::string getHeader() const override {

                    // Get current date
                    auto now = std::chrono::system_clock::now();
                    auto time = std::chrono::system_clock::to_time_t(now);

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
                    ss << "<p>Date: " << std::put_time(std::localtime(&time), "%Y-%m-%d") << "</p>" << std::endl;
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
                        "margin: 0px 5px 2px 5px; "
                      "} "
                      ".time { "
                         "margin-right: 20px; "
                         "width:80px; "
                      "} "
                      ".level { "
                         "margin-right: 20px; "
                         "width: 90px; "
                      "}"
                      ".category { "
                         "margin-right: 20px; "
                         "width: 100px; "
                         "font-weight: bold; "
                      "}"
                      ".message { "
                        "color: #2e2e2e; "
                        "word-wrap: break-word; "
                        "max-width: 800px; "
                      "} "
                      ".body > .category, .body > .message { "
                        "color: #00994d;"
                      "} "
                      ".world > .category, .world > .message { "
                        "color: #3477DB; "
                      "} "
                      ".joint .category, .joint > .message { "
                        "color: #bf8040; "
                      "} "
                      ".proxyshape .category, .proxyshape > .message { "
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
                virtual std::string format(const time_t& time, const std::string& message,
                                           Level level, Category category) override {

                    std::stringstream ss;

                    ss << "<div class='line " + toLowerCase(getCategoryName(category)) + " " + toLowerCase(getLevelName(level)) + "'>";

                    // Time
                    ss << "<div class='time'>";
                    ss << std::put_time(std::localtime(&time), "%X");
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

                    ss << "</div>";

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
                virtual void write(const time_t& time, const std::string& message, Level level, Category category) = 0;
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
                virtual void write(const time_t& time, const std::string& message, Level level, Category category) override {
                    mFileStream << formatter->format(time, message, level, category) << std::endl << std::flush;
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
                virtual void write(const time_t& time, const std::string& message, Level level, Category category) override {
                    mOutputStream << formatter->format(time, message, level, category) << std::endl << std::flush;
                }
        };


    private:

        // -------------------- Attributes -------------------- //

        /// All the log destinations
        List<Destination*> mDestinations;

        /// Map a log format to the given formatter object
        Map<Format, Formatter*> mFormatters;

        /// Mutex
        std::mutex mMutex;

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

}

// Hash function for struct VerticesPair
namespace std {

  template<> struct hash<reactphysics3d::Logger::Format> {

    size_t operator()(const reactphysics3d::Logger::Format format) const {

        return static_cast<size_t>(format);
    }
  };
}

// Use this macro to log something
#define RP3D_LOG(logger, level, category, message) logger->log(level, category, message)

// If the logging is not enabled
#else

// Empty macro in case logs are not enabled
#define RP3D_LOG(logger, level, category, message)

#endif

#endif
