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
#include "BroadBoundingVolume.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {
    
/*  -------------------------------------------------------------------
    Class AABB :
        This class represents a bounding volume of type "Axis Aligned
        Bounding Box". It's a box where all the edges are always aligned
        with the world coordinate system. The AABB is defined by a center
        point and three extent size in the x,y and z directions.
    -------------------------------------------------------------------
*/
class AABB : public BroadBoundingVolume {
    protected :
        Vector3D center;                // Center point of the AABB
        double extent[3];               // Three extents size in the x, y and z directions
        double originalAABBExtent[3];   // Extents of the original AABB (this is used to update the AABB)

    public :
        AABB(const Vector3D& center, double extentX, double extentY, double extentZ);               // Constructor
        virtual ~AABB();                                                                            // Destructor

        Vector3D getCenter() const;                                                                         // Return the center point
        void setCenter(const Vector3D& center);                                                             // Set the center point
        Vector3D getVertex(uint index) const throw (std::invalid_argument);                                 // Return a vertex of the AABB
        double getExtent(uint index) const throw(std::invalid_argument);                                    // Return an extent value
        void setExtent(uint index, double extent) throw(std::invalid_argument);                             // Set an extent value
        double getMinValueOnAxis(uint axis) const throw(std::invalid_argument);                             // Return the minimum position value on the given axis
        double getMaxValueOnAxis(uint axis) const throw(std::invalid_argument);                             // Return the maximum position value on the given axis
        bool testCollision(const AABB& aabb) const;                                                         // Return true if the current AABB is overlapping is the AABB in argument
        virtual void update(const Vector3D& newCenter, const Quaternion& rotationQuaternion);               // Update the oriented bounding box orientation according to a new orientation of the rigid body
        virtual void draw() const;                                                                          // Draw the AABB (only for testing purpose)
        static AABB* computeFromVertices(const std::vector<Vector3D>& vertices, const Vector3D& center);    // Compute an AABB from a set of vertices
};

// Return the center point
inline Vector3D AABB::getCenter() const {
    return center;
}

// Set the center point
inline void AABB::setCenter(const Vector3D& center) {
    this->center = center;
}

// Return one of the 8 vertices of the AABB
inline Vector3D AABB::getVertex(unsigned int index) const throw (std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <8) {
        switch(index) {
            case 0 : return center +  Vector3D(extent[0], extent[1], -extent[2]);
            case 1 : return center + Vector3D(extent[0], extent[1], extent[2]);
            case 2 : return center + Vector3D(-extent[0], extent[1], extent[2]);
            case 3 : return center + Vector3D(-extent[0], extent[1], -extent[2]);
            case 4 : return center + Vector3D(extent[0], -extent[1], -extent[2]);
            case 5 : return center + Vector3D(extent[0], -extent[1], extent[2]);
            case 6 : return center + Vector3D(-extent[0], -extent[1], extent[2]);
            case 7 : return center + Vector3D(-extent[0], -extent[1], -extent[2]);
        }
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 8");
    }
}


// Return an extent value
inline double AABB::getExtent(unsigned int index) const throw(std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <3) {
        return extent[index];
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Set an extent value
inline void AABB::setExtent(unsigned int index, double extent) throw(std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <3) {
        this->extent[index] = extent;
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Return the minimum position value on the given axis
inline double AABB::getMinValueOnAxis(uint axis) const throw(std::invalid_argument) {
    switch (axis) {
        case 0: return center.getX() - extent[0];
        case 1: return center.getY() - extent[1];
        case 2: return center.getZ() - extent[2];
        default: // The index value is not valid, we throw an exception
                 throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Return the maximum position value on the given axis
inline double AABB::getMaxValueOnAxis(uint axis) const throw(std::invalid_argument) {
    switch (axis) {
        case 0: return center.getX() + extent[0];
        case 1: return center.getY() + extent[1];
        case 2: return center.getZ() + extent[2];
        default: // The index value is not valid, we throw an exception
                 throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Return true if the current AABB is overlapping is the AABB in argument
// Two AABB overlap if they overlap in the three x, y and z axis at the same time
inline bool AABB::testCollision(const AABB& aabb) const {
    Vector3D center2 = aabb.getCenter();
    if (std::abs(center.getX() - center2.getX()) > (extent[0] + aabb.getExtent(0))) return false;
    if (std::abs(center.getY() - center2.getY()) > (extent[1] + aabb.getExtent(1))) return false;
    if (std::abs(center.getZ() - center2.getZ()) > (extent[2] + aabb.getExtent(2))) return false;
    return true;
}

// Update the orientation of the AABB according to the orientation of the rigid body
// In order to compute the new AABB we use the original AABB (represented by the originalAABBExtent
// values). The goal is to rotate the original AABB according to the current rotation (rotationQuaternion)
// and then compute the new extent values from the rotated axis of the original AABB. The three columns of
// the rotation matrix correspond to the rotated axis of the rotated original AABB. The we have to compute
// the projections of the three rotated axis onto the x, y and z axis. The projections are easy to compute
// because for instance if the size of the projection of the vector (4, 5, 6) onto the x axis is simply 4.
inline void AABB::update(const Vector3D& newCenter, const Quaternion& rotationQuaternion) {
    // Update the center of the AABB
    center = newCenter;
    
    // Recompute the new extents size from the rotated AABB
    Matrix rotationMatrix = rotationQuaternion.getMatrix();     // Rotation matrix
    for (int i=0; i<3; i++) {   // For each x, y and z axis
        extent[i] = 0.0;
        for (int j=0; j<3; j++) { // For each rotated axis of the rotated original AABB
            // Add the size of the projection of the current rotated axis to the extent of the current (x, y, or z) axis
            extent[i] += std::abs(rotationMatrix.getValue(i, j)) * originalAABBExtent[j];
        }
    }
}

}; // End of the ReactPhysics3D namespace

#endif
