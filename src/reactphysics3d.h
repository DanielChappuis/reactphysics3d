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


/********************************************************************************
* ReactPhysics3D                                                                *
* Version 0.7.1                                                                 *
* http://www.reactphysics3d.com                                                 *
* Daniel Chappuis                                                               *
********************************************************************************/

#ifndef REACTPHYSICS3D_H
#define REACTPHYSICS3D_H

// Libraries
#include "configuration.h"
#include "mathematics/mathematics.h"
#include "body/CollisionBody.h"
#include "body/RigidBody.h"
#include "engine/DynamicsWorld.h"
#include "engine/CollisionWorld.h"
#include "engine/Material.h"
#include "engine/EventListener.h"
#include "collision/shapes/CollisionShape.h"
#include "collision/shapes/BoxShape.h"
#include "collision/shapes/SphereShape.h"
#include "collision/shapes/CapsuleShape.h"
#include "collision/shapes/ConvexMeshShape.h"
#include "collision/shapes/ConcaveMeshShape.h"
#include "collision/shapes/HeightFieldShape.h"
#include "collision/PolyhedronMesh.h"
#include "collision/shapes/AABB.h"
#include "collision/ProxyShape.h"
#include "collision/RaycastInfo.h"
#include "collision/TriangleMesh.h"
#include "collision/PolyhedronMesh.h"
#include "collision/TriangleVertexArray.h"
#include "collision/PolygonVertexArray.h"
#include "collision/CollisionCallback.h"
#include "collision/OverlapCallback.h"
#include "constraint/BallAndSocketJoint.h"
#include "constraint/SliderJoint.h"
#include "constraint/HingeJoint.h"
#include "constraint/FixedJoint.h"
#include "containers/List.h"

/// Alias to the ReactPhysics3D namespace
namespace rp3d = reactphysics3d;

#endif
