/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_PROFILER_H
#define REACTPHYSICS3D_PROFILER_H

// If profiling is enabled
#ifdef IS_RP3D_PROFILING_ENABLED

// Libraries
#include <reactphysics3d/configuration.h>
#include <fstream>
#include <chrono>
#include <reactphysics3d/containers/Array.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

using clock = std::chrono::high_resolution_clock;

// Class ProfileNode
/**
 * It represents a profile sample in the profiler tree.
 */
class ProfileNode {

    private :

        // -------------------- Attributes -------------------- //

        /// Name of the node
        const char* mName;

        /// Total number of calls of this node
        uint mNbTotalCalls;

        /// Starting time of the sampling of corresponding block of code
        std::chrono::time_point<clock> mStartingTime;

        /// Total time spent in the block of code
        std::chrono::duration<double, std::milli> mTotalTime;

        /// Recursion counter
        int mRecursionCounter;

        /// Pointer to the parent node
        ProfileNode* mParentNode;

        /// Pointer to a child node
        ProfileNode* mChildNode;

        /// Pointer to a sibling node
        ProfileNode* mSiblingNode;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ProfileNode(const char* name, ProfileNode* parentNode);

        /// Destructor
        ~ProfileNode();

        /// Return a pointer to a sub node
        ProfileNode* findSubNode(const char* name);

        /// Return a pointer to the parent node
        ProfileNode* getParentNode();

        /// Return a pointer to a sibling node
        ProfileNode* getSiblingNode();

        /// Return a pointer to a child node
        ProfileNode* getChildNode();

        /// Return the name of the node
        const char* getName();

        /// Return the total number of call of the corresponding block of code
        uint getNbTotalCalls() const;

        /// Return the total time spent in the block of code
        std::chrono::duration<double, std::milli> getTotalTime() const;

        /// Called when we enter the block of code corresponding to this profile node
        void enterBlockOfCode();

        /// Called when we exit the block of code corresponding to this profile node
        bool exitBlockOfCode();

        /// Reset the profiling of the node
        void reset();

        /// Destroy the node
        void destroy();
};

// Class ProfileNodeIterator
/**
 * This class allows us to iterator over the profiler tree.
 */
class ProfileNodeIterator {

    private :

        // -------------------- Attributes -------------------- //

        /// Current parent node
        ProfileNode* mCurrentParentNode;

        /// Current child node
        ProfileNode* mCurrentChildNode;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ProfileNodeIterator(ProfileNode* startingNode);

        /// Go to the first node
        void first();

        /// Go to the next node
        void next();

        /// Enter a given child node
        void enterChild(int index);

        /// Enter a given parent node
        void enterParent();

        /// Return true if we are at the root of the profiler tree
        bool isRoot();

        /// Return true if we are at the end of a branch of the profiler tree
        bool isEnd();

        /// Return the name of the current node
        const char* getCurrentName();

        /// Return the total time of the current node
        std::chrono::duration<double, std::milli> getCurrentTotalTime();

        /// Return the total number of calls of the current node
        uint getCurrentNbTotalCalls();

        /// Return the name of the current parent node
        const char* getCurrentParentName();

        /// Return the total time of the current parent node
        std::chrono::duration<double, std::milli> getCurrentParentTotalTime();

        /// Return the total number of calls of the current parent node
        uint getCurrentParentNbTotalCalls();
};

// Class Profiler
/**
 * This is the main class of the profiler. This profiler is based on "Real-Time Hierarchical
 * Profiling" article from "Game Programming Gems 3" by Greg Hjelstrom and Byon Garrabrant.
 */
class Profiler {

    public:

        /// Format of the profiling data (text, ...)
        enum class Format {Text};

        /// Profile destination
        class Destination {

            public:

                /// Log format (text, ...)
                Format format;

                /// Constructor
                Destination(Format logFormat) : format(logFormat) {

                }

                /// Destructor
                virtual ~Destination() {

                }

                /// Return the current output stream
                virtual std::ostream& getOutputStream() = 0;
        };

        /// File destination to output profiling data into a file
        class FileDestination : public Destination {

            private:

                std::string mFilePath;

                /// Output file stream
                std::ofstream mFileStream;

            public:

                /// Constructor
                FileDestination(const std::string& filePath, Format format)
                   :Destination(format), mFilePath(filePath),
                    mFileStream(filePath, std::ios::binary) {

                    if(!mFileStream.is_open()) {
                        throw(std::runtime_error("ReactPhysics3D Logger: Unable to open an output stream to file " + mFilePath));
                    }
                }

                /// Destructor
                virtual ~FileDestination() override {

                    if (mFileStream.is_open()) {

                        // Close the stream
                        mFileStream.close();
                    }
                }

                /// Return the current output stream
                virtual std::ostream& getOutputStream() override {
                    return mFileStream;
                }
        };

        /// Stream destination to output profiling data into a stream
        class StreamDestination : public Destination {

            private:

                /// Output stream
                std::ostream& mOutputStream;

            public:

                /// Constructor
                StreamDestination(std::ostream& outputStream, Format format)
                   :Destination(format), mOutputStream(outputStream) {

                }

                /// Destructor
                virtual ~StreamDestination() override {

                }

                /// Return the current output stream
                virtual std::ostream& getOutputStream() override {
                    return mOutputStream;
                }
        };

    private :

        // -------------------- Attributes -------------------- //

        /// Root node of the profiler tree
        ProfileNode mRootNode;

        /// Current node in the current execution
        ProfileNode* mCurrentNode;

        /// Frame counter
        uint mFrameCounter;

        /// Starting profiling time
        std::chrono::time_point<clock> mProfilingStartTime;

        /// Number of allocated destinations
        uint mNbAllocatedDestinations;

        /// Number of destinations
        uint mNbDestinations;

        /// Array with all the output destinations
        Destination** mDestinations;

        /// Recursively print the report of a given node of the profiler tree
        void printRecursiveNodeReport(ProfileNodeIterator* iterator,  int spacing,
                                      std::ostream &outputStream);

		/// Destroy a previously allocated iterator
		void destroyIterator(ProfileNodeIterator* iterator);

		/// Destroy the profiler (release the memory)
		void destroy();


    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Profiler();

        /// Destructor
        ~Profiler();

        /// Method called when we want to start profiling a block of code.
        void startProfilingBlock(const char *name);

        /// Method called at the end of the scope where the
        /// startProfilingBlock() method has been called.
        void stopProfilingBlock();

        /// Reset the timing data of the profiler (but not the profiler tree structure)
        void reset();

        /// Return the number of frames
        uint getNbFrames();

        /// Return the elasped time since the start/reset of the profiling
        std::chrono::duration<double, std::milli> getElapsedTimeSinceStart();

        /// Increment the frame counter
        void incrementFrameCounter();

        /// Return an iterator over the profiler tree starting at the root
        ProfileNodeIterator* getIterator();

        // Allocate memory for the destinations
        void allocatedDestinations(uint nbDestinationsToAllocate);

        // Add a file destination to the profiler
        void addFileDestination(const std::string& filePath, Format format);

        // Add a stream destination to the profiler
        void addStreamDestination(std::ostream& outputStream, Format format);

        /// Remove all logs destination previously set
        void removeAllDestinations();

        /// Print the report of the profiler in every output destinations
        void printReport();

        // ---------- Friendship ---------- //
        friend class PhysicsCommon;
};

// Class ProfileSample
/**
 * This class is used to represent a profile sample. It is constructed at the
 * beginning of a code block we want to profile and destructed at the end of the
 * scope to profile.
 */
class ProfileSample {

	private:

		Profiler* mProfiler;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ProfileSample(const char* name, Profiler* profiler) :mProfiler(profiler) {

			assert(profiler != nullptr);

            // Ask the profiler to start profiling a block of code
			mProfiler->startProfilingBlock(name);
        }

        /// Destructor
        ~ProfileSample() {

            // Tell the profiler to stop profiling a block of code
			mProfiler->stopProfilingBlock();
        }
};


// Use this macro to start profile a block of code
#define RP3D_PROFILE(name, profiler) ProfileSample profileSample(name, profiler)

// Return true if we are at the root of the profiler tree
RP3D_FORCE_INLINE bool ProfileNodeIterator::isRoot() {
    return (mCurrentParentNode->getParentNode() == nullptr);
}

// Return true if we are at the end of a branch of the profiler tree
RP3D_FORCE_INLINE bool ProfileNodeIterator::isEnd() {
    return (mCurrentChildNode == nullptr);
}

// Return the name of the current node
RP3D_FORCE_INLINE const char* ProfileNodeIterator::getCurrentName() {
    return mCurrentChildNode->getName();
}

// Return the total time of the current node
RP3D_FORCE_INLINE std::chrono::duration<double, std::milli> ProfileNodeIterator::getCurrentTotalTime() {
    return mCurrentChildNode->getTotalTime();
}

// Return the total number of calls of the current node
RP3D_FORCE_INLINE uint ProfileNodeIterator::getCurrentNbTotalCalls() {
    return mCurrentChildNode->getNbTotalCalls();
}

// Return the name of the current parent node
RP3D_FORCE_INLINE const char* ProfileNodeIterator::getCurrentParentName() {
    return mCurrentParentNode->getName();
}

// Return the total time of the current parent node
RP3D_FORCE_INLINE std::chrono::duration<double, std::milli> ProfileNodeIterator::getCurrentParentTotalTime() {
    return mCurrentParentNode->getTotalTime();
}

// Return the total number of calls of the current parent node
RP3D_FORCE_INLINE uint ProfileNodeIterator::getCurrentParentNbTotalCalls() {
    return mCurrentParentNode->getNbTotalCalls();
}

// Go to the first node
RP3D_FORCE_INLINE void ProfileNodeIterator::first() {
    mCurrentChildNode = mCurrentParentNode->getChildNode();
}

// Go to the next node
RP3D_FORCE_INLINE void ProfileNodeIterator::next() {
    mCurrentChildNode = mCurrentChildNode->getSiblingNode();
}

// Return a pointer to the parent node
RP3D_FORCE_INLINE ProfileNode* ProfileNode::getParentNode() {
    return mParentNode;
}

// Return a pointer to a sibling node
RP3D_FORCE_INLINE ProfileNode* ProfileNode::getSiblingNode() {
    return mSiblingNode;
}

// Return a pointer to a child node
RP3D_FORCE_INLINE ProfileNode* ProfileNode::getChildNode() {
    return mChildNode;
}

// Return the name of the node
RP3D_FORCE_INLINE const char* ProfileNode::getName() {
    return mName;
}

// Return the total number of call of the corresponding block of code
RP3D_FORCE_INLINE uint ProfileNode::getNbTotalCalls() const {
    return mNbTotalCalls;
}

// Return the total time spent in the block of code
RP3D_FORCE_INLINE std::chrono::duration<double, std::milli> ProfileNode::getTotalTime() const {
    return mTotalTime;
}

// Return the number of frames
RP3D_FORCE_INLINE uint Profiler::getNbFrames() {
    return mFrameCounter;
}

// Return the elasped time since the start/reset of the profiling
RP3D_FORCE_INLINE std::chrono::duration<double, std::milli> Profiler::getElapsedTimeSinceStart() {
    return (clock::now() - mProfilingStartTime);
}

// Increment the frame counter
RP3D_FORCE_INLINE void Profiler::incrementFrameCounter() {
    mFrameCounter++;
}

// Return an iterator over the profiler tree starting at the root
RP3D_FORCE_INLINE ProfileNodeIterator* Profiler::getIterator() {
    return new ProfileNodeIterator(&mRootNode);
}

// Destroy a previously allocated iterator
RP3D_FORCE_INLINE void Profiler::destroyIterator(ProfileNodeIterator* iterator) {
    delete iterator;
}

// Destroy the profiler (release the memory)
RP3D_FORCE_INLINE void Profiler::destroy() {
    mRootNode.destroy();
}

}

// If profiling is disabled
#else

// Empty macro in case profiling is not active
#define RP3D_PROFILE(name, profiler)

#endif

#endif
