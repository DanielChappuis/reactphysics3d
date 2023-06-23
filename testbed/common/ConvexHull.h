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

#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "ConvexMesh.h"

// Class ConvexHull
// Created from any triangular mesh this object will have a collider
// that is the convex hull of the input triangular mesh
class ConvexHull : public ConvexMesh {

    private :

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexHull(reactphysics3d::BodyType type, bool isSimulationCollider, rp3d::PhysicsCommon& physicsCommon, rp3d::PhysicsWorld* physicsWorld,
                   const std::string& meshPath, const rp3d::Vector3& scaling = rp3d::Vector3(1, 1, 1));
};

#endif
