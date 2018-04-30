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

// Libraries
#include "ConvexPolyhedronShape.h"


// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
ConvexPolyhedronShape::ConvexPolyhedronShape(CollisionShapeName name)
            : ConvexShape(name, CollisionShapeType::CONVEX_POLYHEDRON) {

}

// Find and return the index of the polyhedron face with the most anti-parallel face
// normal given a direction vector. This is used to find the incident face on
// a polyhedron of a given reference face of another polyhedron
uint ConvexPolyhedronShape::findMostAntiParallelFace(const Vector3& direction) const {

    decimal minDotProduct = DECIMAL_LARGEST;
    uint mostAntiParallelFace = 0;

    // For each face of the polyhedron
    for (uint i=0; i < getNbFaces(); i++) {

        // Get the face normal
        decimal dotProduct = getFaceNormal(i).dot(direction);
        if (dotProduct < minDotProduct) {
            minDotProduct = dotProduct;
            mostAntiParallelFace = i;
        }
    }

    return mostAntiParallelFace;
}
