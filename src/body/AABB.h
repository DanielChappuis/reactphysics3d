/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

#ifndef AABB_H
#define AABB_H

// Libraries
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Declaration
class Body;
    
/*  -------------------------------------------------------------------
    Class AABB :
        This class represents a bounding volume of type "Axis Aligned
        Bounding Box". It's a box where all the edges are always aligned
        with the world coordinate system. The AABB is defined by a center
        point and three extent size in the x,y and z directions.
    -------------------------------------------------------------------
*/
class AABB {
    private :
        Vector3D minCoordinates;        // Minimum world coordinates of the AABB on the x,y and z axis
        Vector3D maxCoordinates;        // Maximum world coordinates of the AABB on the x,y and z axis
        Body* bodyPointer;              // Pointer to the owner body (not the abstract class Body but its derivative which is instanciable)

    public :
        AABB();                                                           // Constructor
        AABB(const Transform& transform, const Vector3D& extents);        // Constructor
        virtual ~AABB();                                                  // Destructor

        Vector3D getCenter() const;                                                     // Return the center point
        const Vector3D& getMinCoordinates() const;                                      // Return the minimum coordinates of the AABB
        const Vector3D& getMaxCoordinates() const;                                      // Return the maximum coordinates of the AABB
        Body* getBodyPointer() const;                                                   // Return a pointer to the owner body
        void setBodyPointer(Body* bodyPointer);                                         // Set the body pointer
        bool testCollision(const AABB& aabb) const;                                     // Return true if the current AABB is overlapping is the AABB in argument
        virtual void update(const Transform& newTransform, const Vector3D& extents);    // Update the oriented bounding box orientation according to a new orientation of the rigid body
        #ifdef VISUAL_DEBUG
           virtual void draw() const;                                                               // Draw the AABB (only for testing purpose)
        #endif
};

// Return the center point of the AABB in world coordinates
inline Vector3D AABB::getCenter() const {
    return (minCoordinates + maxCoordinates) * 0.5;
}

// Return the minimum coordinates of the AABB
inline const Vector3D& AABB::getMinCoordinates() const {
    return minCoordinates;
}

// Return the maximum coordinates of the AABB
inline const Vector3D& AABB::getMaxCoordinates() const {
    return maxCoordinates;
}

// Return a pointer to the owner body
inline Body* AABB::getBodyPointer() const {
    return bodyPointer;
}

// Set the body pointer
inline void AABB::setBodyPointer(Body* bodyPointer) {
    this->bodyPointer = bodyPointer;
}

// Return true if the current AABB is overlapping with the AABB in argument
// Two AABB overlap if they overlap in the three x, y and z axis at the same time
inline bool AABB::testCollision(const AABB& aabb) const {
    if (maxCoordinates.getX() + OBJECT_MARGIN < aabb.minCoordinates.getX() - OBJECT_MARGIN || 
        aabb.maxCoordinates.getX() + OBJECT_MARGIN < minCoordinates.getX() - OBJECT_MARGIN) return false;
    if (maxCoordinates.getY() + OBJECT_MARGIN < aabb.minCoordinates.getY() - OBJECT_MARGIN ||
        aabb.maxCoordinates.getY() + OBJECT_MARGIN < minCoordinates.getY() - OBJECT_MARGIN) return false;
    if (maxCoordinates.getZ() + OBJECT_MARGIN < aabb.minCoordinates.getZ() - OBJECT_MARGIN ||
        aabb.maxCoordinates.getZ() + OBJECT_MARGIN < minCoordinates.getZ() - OBJECT_MARGIN) return false;
    return true;
}

// Update the world minimum and maximum coordinates of the AABB on the three x,y and z axis
inline void AABB::update(const Transform& newTransform, const Vector3D& extents) {
    Vector3D localExtents = extents + Vector3D(OBJECT_MARGIN, OBJECT_MARGIN, OBJECT_MARGIN);
    Matrix3x3 worldAxis = newTransform.getOrientation().getAbsoluteMatrix();
    Vector3D worldExtents = Vector3D(worldAxis.getColumn(0).dot(localExtents),
                                     worldAxis.getColumn(1).dot(localExtents),
                                     worldAxis.getColumn(2).dot(localExtents));
    minCoordinates = newTransform.getPosition() - worldExtents;
    maxCoordinates = newTransform.getPosition() + worldExtents;
}

}; // End of the ReactPhysics3D namespace

#endif
