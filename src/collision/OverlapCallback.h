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

#ifndef REACTPHYSICS3D_OVERLAP_CALLBACK_H
#define REACTPHYSICS3D_OVERLAP_CALLBACK_H

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionBody;

// Class OverlapCallback
/**
 * This class can be used to register a callback for collision overlap queries.
 * You should implement your own class inherited from this one and implement
 * the notifyOverlap() method. This method will called each time a contact
 * point is reported.
 */
class OverlapCallback {

    public:

        /// Destructor
        virtual ~OverlapCallback() {

        }

        /// This method will be called for each reported overlapping bodies
        virtual void notifyOverlap(CollisionBody* collisionBody)=0;
};

}

#endif
