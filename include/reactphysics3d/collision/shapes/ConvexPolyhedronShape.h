/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONVEX_POLYHEDRON_H
#define REACTPHYSICS3D_CONVEX_POLYHEDRON_H

// Libraries
#include <reactphysics3d/collision/shapes/ConvexShape.h>
#include <reactphysics3d/collision/HalfEdgeStructure.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class ConvexPolyhedronShape
/**
 * This abstract class represents a convex polyhedron collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ConvexPolyhedronShape : public ConvexShape {

    protected :

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexPolyhedronShape(CollisionShapeName name, MemoryAllocator& allocator);

        /// Destructor
        virtual ~ConvexPolyhedronShape() override = default;

        /// Deleted copy-constructor
        ConvexPolyhedronShape(const ConvexPolyhedronShape& shape) = delete;

        /// Deleted assignment operator
        ConvexPolyhedronShape& operator=(const ConvexPolyhedronShape& shape) = delete;

        /// Return the number of faces of the polyhedron
        virtual uint32 getNbFaces() const=0;

        /// Return a given face of the polyhedron
        virtual const HalfEdgeStructure::Face& getFace(uint32 faceIndex) const=0;

        /// Return the number of vertices of the polyhedron
        virtual uint32 getNbVertices() const=0;

        /// Return a given vertex of the polyhedron
        virtual const HalfEdgeStructure::Vertex& getVertex(uint32 vertexIndex) const=0;

        /// Return the position of a given vertex
        virtual Vector3 getVertexPosition(uint32 vertexIndex) const=0;

        /// Return the normal vector of a given face of the polyhedron
        virtual Vector3 getFaceNormal(uint32 faceIndex) const=0;

        /// Return the number of half-edges of the polyhedron
        virtual uint32 getNbHalfEdges() const=0;

        /// Return a given half-edge of the polyhedron
        virtual const HalfEdgeStructure::Edge& getHalfEdge(uint32 edgeIndex) const=0;

        /// Return true if the collision shape is a polyhedron
        virtual bool isPolyhedron() const override;

        /// Return the centroid of the polyhedron
        virtual Vector3 getCentroid() const=0;

        /// Find and return the index of the polyhedron face with the most anti-parallel face
        /// normal given a direction vector
        uint32 findMostAntiParallelFace(const Vector3& direction) const;
};

// Return true if the collision shape is a polyhedron
RP3D_FORCE_INLINE bool ConvexPolyhedronShape::isPolyhedron() const {
    return true;
}


// Find and return the index of the polyhedron face with the most anti-parallel face
// normal given a direction vector. This is used to find the incident face on
// a polyhedron of a given reference face of another polyhedron
RP3D_FORCE_INLINE uint32 ConvexPolyhedronShape::findMostAntiParallelFace(const Vector3& direction) const {

    decimal minDotProduct = DECIMAL_LARGEST;
    uint32 mostAntiParallelFace = 0;

    // For each face of the polyhedron
    const uint32 nbFaces = getNbFaces();
    for (uint32 i=0; i < nbFaces; i++) {

        // Get the face normal
        const decimal dotProduct = getFaceNormal(i).dot(direction);
        if (dotProduct < minDotProduct) {
            minDotProduct = dotProduct;
            mostAntiParallelFace = i;
        }
    }

    return mostAntiParallelFace;
}

}

#endif

