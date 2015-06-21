/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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

#ifdef IS_PROFILING_ACTIVE

// Libraries
#include "reactphyiscs3d/configuration.h"
#include "Timer.h" // ???

/// ReactPhysics3D namespace
namespace reactphysics3d {

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
        long double mStartingTime;

        /// Total time spent in the block of code
        long double mTotalTime;

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
        long double getTotalTime() const;

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
        long double getCurrentTotalTime();

        /// Return the total number of calls of the current node
        uint getCurrentNbTotalCalls();

        /// Return the name of the current parent node
        const char* getCurrentParentName();

        /// Return the total time of the current parent node
        long double getCurrentParentTotalTime();

        /// Return the total number of calls of the current parent node
        uint getCurrentParentNbTotalCalls();
};

// Class Profiler
/**
 * This is the main class of the profiler. This profiler is based on "Real-Time Hierarchical
 * Profiling" article from "Game Programming Gems 3" by Greg Hjelstrom and Byon Garrabrant.
 */
class Profiler {

    private :

        // -------------------- Attributes -------------------- //

        /// Root node of the profiler tree
        static ProfileNode mRootNode;

        /// Current node in the current execution
        static ProfileNode* mCurrentNode;

        /// Frame counter
        static uint mFrameCounter;

        /// Starting profiling time
        static long double mProfilingStartTime;

        /// Recursively print the report of a given node of the profiler tree
        static void printRecursiveNodeReport(ProfileNodeIterator* iterator,
                                             int spacing,
                                             std::ostream& outputStream);

    public :

        // -------------------- Methods -------------------- //

        /// Method called when we want to start profiling a block of code.
        static void startProfilingBlock(const char *name);

        /// Method called at the end of the scope where the
        /// startProfilingBlock() method has been called.
        static void stopProfilingBlock();

        /// Reset the timing data of the profiler (but not the profiler tree structure)
        static void reset();

        /// Return the number of frames
        static uint getNbFrames();

        /// Return the elasped time since the start/reset of the profiling
        static long double getElapsedTimeSinceStart();

        /// Increment the frame counter
        static void incrementFrameCounter();

        /// Return an iterator over the profiler tree starting at the root
        static ProfileNodeIterator* getIterator();

        /// Print the report of the profiler in a given output stream
        static void printReport(std::ostream& outputStream);

        /// Destroy a previously allocated iterator
        static void destroyIterator(ProfileNodeIterator* iterator);

        /// Destroy the profiler (release the memory)
        static void destroy();
};

// Class ProfileSample
/**
 * This class is used to represent a profile sample. It is constructed at the
 * beginning of a code block we want to profile and destructed at the end of the
 * scope to profile.
 */
class ProfileSample {

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ProfileSample(const char* name) {

            // Ask the profiler to start profiling a block of code
            Profiler::startProfilingBlock(name);
        }

        /// Destructor
        ~ProfileSample() {

            // Tell the profiler to stop profiling a block of code
            Profiler::stopProfilingBlock();
        }
};

// Use this macro to start profile a block of code
#define PROFILE(name) ProfileSample profileSample(name)

// Return true if we are at the root of the profiler tree
inline bool ProfileNodeIterator::isRoot() {
    return (mCurrentParentNode->getParentNode() == NULL);
}

// Return true if we are at the end of a branch of the profiler tree
inline bool ProfileNodeIterator::isEnd() {
    return (mCurrentChildNode == NULL);
}

// Return the name of the current node
inline const char* ProfileNodeIterator::getCurrentName() {
    return mCurrentChildNode->getName();
}

// Return the total time of the current node
inline long double ProfileNodeIterator::getCurrentTotalTime() {
    return mCurrentChildNode->getTotalTime();
}

// Return the total number of calls of the current node
inline uint ProfileNodeIterator::getCurrentNbTotalCalls() {
    return mCurrentChildNode->getNbTotalCalls();
}

// Return the name of the current parent node
inline const char* ProfileNodeIterator::getCurrentParentName() {
    return mCurrentParentNode->getName();
}

// Return the total time of the current parent node
inline long double ProfileNodeIterator::getCurrentParentTotalTime() {
    return mCurrentParentNode->getTotalTime();
}

// Return the total number of calls of the current parent node
inline uint ProfileNodeIterator::getCurrentParentNbTotalCalls() {
    return mCurrentParentNode->getNbTotalCalls();
}

// Go to the first node
inline void ProfileNodeIterator::first() {
    mCurrentChildNode = mCurrentParentNode->getChildNode();
}

// Go to the next node
inline void ProfileNodeIterator::next() {
    mCurrentChildNode = mCurrentChildNode->getSiblingNode();
}

// Return a pointer to the parent node
inline ProfileNode* ProfileNode::getParentNode() {
    return mParentNode;
}

// Return a pointer to a sibling node
inline ProfileNode* ProfileNode::getSiblingNode() {
    return mSiblingNode;
}

// Return a pointer to a child node
inline ProfileNode* ProfileNode::getChildNode() {
    return mChildNode;
}

// Return the name of the node
inline const char* ProfileNode::getName() {
    return mName;
}

// Return the total number of call of the corresponding block of code
inline uint ProfileNode::getNbTotalCalls() const {
    return mNbTotalCalls;
}

// Return the total time spent in the block of code
inline long double ProfileNode::getTotalTime() const {
    return mTotalTime;
}

// Return the number of frames
inline uint Profiler::getNbFrames() {
    return mFrameCounter;
}

// Return the elasped time since the start/reset of the profiling
inline long double Profiler::getElapsedTimeSinceStart() {
    long double currentTime = Timer::getCurrentSystemTime() * 1000.0;
    return currentTime - mProfilingStartTime;
}

// Increment the frame counter
inline void Profiler::incrementFrameCounter() {
    mFrameCounter++;
}

// Return an iterator over the profiler tree starting at the root
inline ProfileNodeIterator* Profiler::getIterator() {
    return new ProfileNodeIterator(&mRootNode);
}

// Destroy a previously allocated iterator
inline void Profiler::destroyIterator(ProfileNodeIterator* iterator) {
    delete iterator;
}

// Destroy the profiler (release the memory)
inline void Profiler::destroy() {
    mRootNode.destroy();
}

}

#else   // In profile is not active

// Empty macro in case profiling is not active
#define PROFILE(name)

#endif

#endif
