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

#ifndef OBB_H
#define OBB_H

// Libraries
#include <cfloat>
#include "NarrowBoundingVolume.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class OBB :
        This class represents a bounding volume of type "Oriented Bounding
        Box". It's a box that has a given orientation is space. The three
        axis of the OBB are three axis that give the orientation of the
        OBB. The three axis are normal vectors to the faces of the OBB.
        Those axis are unit length. The three extents are half-widths
        of the box along the three axis of the OBB.
    -------------------------------------------------------------------
*/
class OBB : public NarrowBoundingVolume {
    protected :
        Vector3D center;            // Center point of the OBB
        Vector3D oldAxis[3];        // Array that contains the three unit length axis at the beginning
        Vector3D axis[3];           // Array that contains the three unit length axis of the OBB
        double extent[3];           // Array that contains the three extents size of the OBB

    public :
        OBB(const Vector3D& center, const Vector3D& axis1, const Vector3D& axis2,
            const Vector3D& axis3, double extent1, double extent2, double extent3);                         // Constructor
        virtual ~OBB();                                                                                     // Destructor

        Vector3D getCenter() const;                                                                         // Return the center point of the OBB
        void setCenter(const Vector3D& center);                                                             // Set the center point
        Vector3D getAxis(unsigned int index) const throw(std::invalid_argument);                            // Return an axis of the OBB
        void setAxis(unsigned int index, const Vector3D& axis) throw(std::invalid_argument);                // Set an axis
        Vector3D getVertex(unsigned int index) const throw (std::invalid_argument);                         // Return a vertex of the OBB
        std::vector<Vector3D> getFace(unsigned int index) const throw(std::invalid_argument);               // Return the 4 vertices the OBB's face in the direction of a given axis
        double getExtent(unsigned int index) const throw(std::invalid_argument);                            // Return an extent value
        void setExtent(unsigned int index, double extent) throw(std::invalid_argument);                     // Set an extent value
        virtual std::vector<Vector3D> getExtremeVertices(const Vector3D& axis) const;                       // Return all the vertices that are projected at the extreme of the projection of the bouding volume on the axis
        virtual void update(const Vector3D& newCenter, const Quaternion& rotationQuaternion);               // Update the oriented bounding box orientation according to a new orientation of the rigid body
        virtual AABB* computeAABB() const;                                                                  // Return the corresponding AABB
        virtual Vector3D getSupportPoint(const Vector3D& direction) const;                                  // Return a support point in a given direction

#ifdef VISUAL_DEBUG
            virtual void draw() const;                                                                      // Draw the OBB (only for testing purpose)
#endif
        static OBB* computeFromVertices(const std::vector<Vector3D>& vertices, const Vector3D& center);     // Compute an OBB from a set of vertices
};

// Return the center point of the OBB
inline Vector3D OBB::getCenter() const {
    return center;
}

// Set the center point
inline void OBB::setCenter(const Vector3D& center) {
    this->center = center;
}

// Return an axis of the OBB
inline Vector3D OBB::getAxis(unsigned int index) const throw(std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <3) {
        return axis[index];
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Set an axis
inline void OBB::setAxis(unsigned int index, const Vector3D& axis) throw(std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <3) {
        this->axis[index] = axis;
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Return a vertex of the OBB
inline Vector3D OBB::getVertex(unsigned int index) const throw (std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <8) {
        Vector3D vertex;

        switch(index) {
            case 0 : vertex = center + (axis[0]*extent[0]) + (axis[1]*extent[1]) - (axis[2]*extent[2]);
                     break;
            case 1 : vertex = center + (axis[0]*extent[0]) + (axis[1]*extent[1]) + (axis[2]*extent[2]);
                     break;
            case 2 : vertex = center - (axis[0]*extent[0]) + (axis[1]*extent[1]) + (axis[2]*extent[2]);
                     break;
            case 3 : vertex = center - (axis[0]*extent[0]) + (axis[1]*extent[1]) - (axis[2]*extent[2]);
                     break;
            case 4 : vertex = center + (axis[0]*extent[0]) - (axis[1]*extent[1]) - (axis[2]*extent[2]);
                     break;
            case 5 : vertex = center + (axis[0]*extent[0]) - (axis[1]*extent[1]) + (axis[2]*extent[2]);
                     break;
            case 6 : vertex = center - (axis[0]*extent[0]) - (axis[1]*extent[1]) + (axis[2]*extent[2]);
                     break;
            case 7 : vertex = center - (axis[0]*extent[0]) - (axis[1]*extent[1]) - (axis[2]*extent[2]);
                     break;
        }

        // Return the vertex
        return vertex;
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 8");
    }
}

// Return the 4 vertices of a face of the OBB. The 4 vertices will be ordered. The convention is that the index 0 corresponds to
// the face in the direction of the axis[0], 1 corresponds to the face in the opposite direction of the axis[0], 2 corresponds to
// the face in the direction of the axis[1], etc.
inline std::vector<Vector3D> OBB::getFace(unsigned int index) const throw(std::invalid_argument) {
    // Check the argument
    if (index >=0 && index <6) {
        std::vector<Vector3D> vertices;
        switch(index) {
            case 0: vertices.push_back(center + (axis[0]*extent[0]) + (axis[1]*extent[1]) - (axis[2]*extent[2]));
                    vertices.push_back(center + (axis[0]*extent[0]) + (axis[1]*extent[1]) + (axis[2]*extent[2]));
                    vertices.push_back(center + (axis[0]*extent[0]) - (axis[1]*extent[1]) + (axis[2]*extent[2]));
                    vertices.push_back(center + (axis[0]*extent[0]) - (axis[1]*extent[1]) - (axis[2]*extent[2]));
                    break;
            case 1: vertices.push_back(center - (axis[0]*extent[0]) + (axis[1]*extent[1]) - (axis[2]*extent[2]));
                    vertices.push_back(center - (axis[0]*extent[0]) + (axis[1]*extent[1]) + (axis[2]*extent[2]));
                    vertices.push_back(center - (axis[0]*extent[0]) - (axis[1]*extent[1]) + (axis[2]*extent[2]));
                    vertices.push_back(center - (axis[0]*extent[0]) - (axis[1]*extent[1]) - (axis[2]*extent[2]));
                    break;
            case 2: vertices.push_back(center + (axis[1]*extent[1]) + (axis[0]*extent[0]) - (axis[2]*extent[2]));
                    vertices.push_back(center + (axis[1]*extent[1]) + (axis[0]*extent[0]) + (axis[2]*extent[2]));
                    vertices.push_back(center + (axis[1]*extent[1]) - (axis[0]*extent[0]) + (axis[2]*extent[2]));
                    vertices.push_back(center + (axis[1]*extent[1]) - (axis[0]*extent[0]) - (axis[2]*extent[2]));
                    break;
            case 3: vertices.push_back(center - (axis[1]*extent[1]) + (axis[0]*extent[0]) - (axis[2]*extent[2]));
                    vertices.push_back(center - (axis[1]*extent[1]) + (axis[0]*extent[0]) + (axis[2]*extent[2]));
                    vertices.push_back(center - (axis[1]*extent[1]) - (axis[0]*extent[0]) + (axis[2]*extent[2]));
                    vertices.push_back(center - (axis[1]*extent[1]) - (axis[0]*extent[0]) - (axis[2]*extent[2]));
                    break;
            case 4: vertices.push_back(center + (axis[2]*extent[2]) + (axis[0]*extent[0]) - (axis[1]*extent[1]));
                    vertices.push_back(center + (axis[2]*extent[2]) + (axis[0]*extent[0]) + (axis[1]*extent[1]));
                    vertices.push_back(center + (axis[2]*extent[2]) - (axis[0]*extent[0]) + (axis[1]*extent[1]));
                    vertices.push_back(center + (axis[2]*extent[2]) - (axis[0]*extent[0]) - (axis[1]*extent[1]));
                    break;
            case 5: vertices.push_back(center - (axis[2]*extent[2]) + (axis[0]*extent[0]) - (axis[1]*extent[1]));
                    vertices.push_back(center - (axis[2]*extent[2]) + (axis[0]*extent[0]) + (axis[1]*extent[1]));
                    vertices.push_back(center - (axis[2]*extent[2]) - (axis[0]*extent[0]) + (axis[1]*extent[1]));
                    vertices.push_back(center - (axis[2]*extent[2]) - (axis[0]*extent[0]) - (axis[1]*extent[1]));
                    break;
        }

        // Return the vertices
        assert(vertices.size() == 4);
        return vertices;
    }
    else {
        // Throw an exception
        throw std::invalid_argument("Exception: The argument must be between 0 and 5");
    }
}

// Return an extent value
inline double OBB::getExtent(unsigned int index) const throw(std::invalid_argument) {
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
inline void OBB::setExtent(unsigned int index, double extent) throw(std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <3) {
        this->extent[index] = extent;
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Update the orientation of the OBB according to the orientation of the rigid body
inline void OBB::update(const Vector3D& newCenter, const Quaternion& rotationQuaternion) {
    // Update the center of the OBB
    center = newCenter;

    // Update the three axis of the OBB by rotating and normalize then
    axis[0] = rotateVectorWithQuaternion(oldAxis[0], rotationQuaternion).getUnit();
    axis[1] = rotateVectorWithQuaternion(oldAxis[1], rotationQuaternion).getUnit();
    axis[2] = rotateVectorWithQuaternion(oldAxis[2], rotationQuaternion).getUnit();
}

// Return a support point in a given direction
inline Vector3D OBB::getSupportPoint(const Vector3D& direction) const {
    // TODO : Implement this method
    assert(false);
    return Vector3D();
}

}; // End of the ReactPhysics3D namespace

#endif
