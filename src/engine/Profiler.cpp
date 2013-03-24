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

#ifdef IS_PROFILING_ACTIVE

// Libraries
#include "Profiler.h"

using namespace reactphysics3d;

// Initialization of static variables
ProfileNode Profiler::mRootNode("Root", NULL);
ProfileNode* Profiler::mCurrentNode = &Profiler::mRootNode;
uint Profiler::mFrameCounter = 0;

// Constructor
ProfileNode::ProfileNode(const char* name, ProfileNode* parentNode)
    :mName(name), mNbTotalCalls(0), mStartingTime(0), mTotalTime(0),
     mRecursionCounter(0), mParentNode(parentNode), mChildNode(NULL),
     mSiblingNode(NULL) {

}

// Destructor
ProfileNode::~ProfileNode() {

    delete mChildNode;
    delete mSiblingNode;
}

// Return a pointer to a sub node with a given name
ProfileNode* ProfileNode::findSubNode(const char* name) {

    // Try to find the node among the child nodes
    ProfileNode* child = mChildNode;
    while (child != NULL) {
        if (child->mName == name) {
            return child;
        }
        child = child->mSiblingNode;
    }

    // The nose has not been found. Therefore, we create it
    // and add it to the profiler tree
    ProfileNode* newNode = new ProfileNode(name, this);
    newNode->mSiblingNode = child;
    child = newNode;

    return newNode;
}

// Called when we enter the block of code corresponding to this profile node
void ProfileNode::enterBlockOfCode() {
    mNbTotalCalls++;

    // If the current code is not called recursively
    if (mRecursionCounter == 0) {

        // Get the current system time to initialize the starting time of
        // the profiling of the current block of code
        mStartingTime = Timer::getCurrentSystemTime();
    }

    mRecursionCounter++;
}

// Called when we exit the block of code corresponding to this profile node
bool ProfileNode::exitBlockOfCode() {
    mRecursionCounter--;

    if (mRecursionCounter == 0 && mNbTotalCalls != 0) {

        // Get the current system time
        long double currentTime = Timer::getCurrentSystemTime();

        // Increase the total elasped time in the current block of code
        mTotalTime += currentTime - mStartingTime;
    }

    // Return true if the current code is not recursing
    return (mRecursionCounter == 0);
}

// Method called when we want to start profiling a block of code.
void Profiler::startProfilingBlock(const char* name) {

    // Look for the node in the tree that corresponds to the block of
    // code to profile
    if (name != mCurrentNode->getName()) {
        mCurrentNode = mCurrentNode->findSubNode(name);
    }

    // Start profile the node
    mCurrentNode->enterBlockOfCode();
}

// Method called at the end of the scope where the
// startProfilingBlock() method has been called.
void Profiler::stopProfilingBlock() {

    // Go to the parent node unless if the current block
    // of code is recursing
    if (mCurrentNode->exitBlockOfCode()) {
        mCurrentNode = mCurrentNode->getParentNode();
    }

}

// Return an iterator over the profiler tree starting at the root
ProfileNodeIterator* Profiler::getIterator() {
    return new ProfileNodeIterator(&mRootNode);
}

// Print the report of the profiler in a given output stream
void printReport(std::ostream& outputStream) {

}

#endif
