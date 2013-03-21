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

// Libraries

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
            Profiler::startProfilingBlock();
        }

        /// Destructor
        ~ProfileSample() {

            // Tell the profiler to stop profiling a block of code
            Profiler::stopProfilingBlock();
        }
};

/// Use this macro to start profile a block of code
#define PROFILE(name) ProfileSample profileSample(name)

// Class ProfileNode
/**
 * Node of the profiler tree
 */
class ProfileNode {

};

// Class Profiler
/**
 * This is the main class of the profiler
 */
class Profiler {

    public :

        /// Method called when we want to start profiling a block of code.
        static void startProfilingBlock() {

        }

        /// Method called at the end of the scope where the startProfilingBlock() method
        /// has been called.
        static void stopProfilingBlock() {

        }
};

#endif
