/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef PROFILER_H
#define PROFILER_H

#ifdef IS_PROFILING_ACTIVE

// Libraries
#include "../configuration.h"
#include "Timer.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class ProfileNode
/**
 * It represents a profile sample in the profiler tree.
 */
class ProfileNode {

    private :

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
};

// Class ProfileNodeIterator
/**
 * This class allow us to iterator over the profiler tree.
 */
class ProfileNodeIterator {

    private :

        /// Current parent node
        ProfileNode* mCurrentParent;

        /// Current child node
        ProfileNode* mCurrentChild;

    public :

        /// Constructor
        ProfileNodeIterator(ProfileNode* startingNode);

        /// Go to the first node
        void first();

        /// Go to the next node
        void next();

        /// Enter a given child node
        void enterChild();

};

// Class Profiler
/**
 * This is the main class of the profiler. This profiler is based on "Real-Time Hierarchical
 * Profiling" article from "Game Programming Gems 3" by Greg Hjelstrom and Byon Garrabrant.
 */
class Profiler {

    private :

        /// Root node of the profiler tree
        static ProfileNode mRootNode;

        /// Current node in the current execution
        static ProfileNode* mCurrentNode;

        /// Frame counter
        static uint mFrameCounter;

    public :

        /// Method called when we want to start profiling a block of code.
        static void startProfilingBlock(const char *name);

        /// Method called at the end of the scope where the
        /// startProfilingBlock() method has been called.
        static void stopProfilingBlock();

        /// Return an iterator over the profiler tree starting at the root
        static ProfileNodeIterator* getIterator();

        /// Print the report of the profiler in a given output stream
        static void printReport(std::ostream& outputStream);
};

// Class ProfileSample
/**
 * This class is used to represent a profile sample. It is constructed at the
 * beginning of a code block we want to profile and destructed at the end of the
 * scope to profile.
 */
class ProfileSample {

    public :

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

/// Use this macro to start profile a block of code
#define PROFILE(name) ProfileSample profileSample(name)

/// Return a pointer to the parent node
ProfileNode* ProfileNode::getParentNode() {
    return mParentNode;
}

/// Return a pointer to a sibling node
ProfileNode* ProfileNode::getSiblingNode() {
    return mSiblingNode;
}

/// Return a pointer to a child node
ProfileNode* ProfileNode::getChildNode() {
    return mChildNode;
}

/// Return the name of the node
const char* ProfileNode::getName() {
    return mName;
}

/// Return the total number of call of the corresponding block of code
uint ProfileNode::getNbTotalCalls() const {
    return mNbTotalCalls;
}

/// Return the total time spent in the block of code
long double ProfileNode::getTotalTime() const {
    return mTotalTime;
}

}

#else   // In profile is not active

// Empty macro in case profiling is not active
#define PROFILE(name)

#endif

#endif
