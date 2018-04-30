/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_COLLISION_CALLBACK_H
#define REACTPHYSICS3D_COLLISION_CALLBACK_H

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class OverlappingPair;
class ContactManifold;
struct ContactManifoldListElement;
class CollisionBody;
class ProxyShape;
class MemoryManager;

// Class CollisionCallback
/**
 * This class can be used to register a callback for collision test queries.
 * You should implement your own class inherited from this one and implement
 * the notifyContact() method. This method will called each time a contact
 * point is reported.
 */
class CollisionCallback {

    public:

        // Structure CollisionCallbackInfo
        /**
         * This structure represents the contact information between two collision
         * shapes of two bodies
         */
        struct CollisionCallbackInfo {

            public:

                /// Pointer to the first element of the linked-list that contains contact manifolds
                ContactManifoldListElement* contactManifoldElements;

                /// Pointer to the first body of the contact
                CollisionBody* body1;

                /// Pointer to the second body of the contact
                CollisionBody* body2;

                /// Pointer to the proxy shape of first body
                const ProxyShape* proxyShape1;

                /// Pointer to the proxy shape of second body
                const ProxyShape* proxyShape2;

                /// Memory manager
                MemoryManager& mMemoryManager;

                // Constructor
                CollisionCallbackInfo(OverlappingPair* pair, MemoryManager& memoryManager);

                // Destructor
                ~CollisionCallbackInfo();
        };

        /// Destructor
        virtual ~CollisionCallback() = default;

        /// This method will be called for each reported contact point
        virtual void notifyContact(const CollisionCallbackInfo& collisionCallbackInfo)=0;
};

}

#endif
