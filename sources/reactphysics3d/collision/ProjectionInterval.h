/****************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU Lesser General Public License as published *
* by the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU Lesser General Public License for more details.                      *
*                                                                          *
* You should have received a copy of the GNU Lesser General Public License *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

#ifndef PROJECTIONINTERVAL_H
#define PROJECTIONINTERVAL_H

// Libraries
#include <vector>
#include "../body/BoundingVolume.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Type of the extreme of an interval. For instance if a extreme of an
// interval is the result of the projection of an edge, the type will be
// EDGE.
enum ExtremeType {VERTEX, EDGE, FACE};

/*  -------------------------------------------------------------------
    Class ProjectionInterval :
        This class represents an projection interval of an bounding
        volume onto a separation axis.
    -------------------------------------------------------------------
*/
class ProjectionInterval {
    private :
        BoundingVolume* boundingVolume;             // Pointer on the bounding volume corresponding to this projection interval
        double min;                                 // Minimum of the interval
        double max;                                 // Maximum of the interval
        ExtremeType minType;                        // Type of the extreme of the projection interval
        ExtremeType maxType;                        // Type of the extreme of the projection interval
        std::vector<Vector3D> minProjectedPoints;   // Projected points onto the minimum of the interval
        std::vector<Vector3D> maxProjectedPoints;   // Projected points onto the maximum of the interval

    public :
        ProjectionInterval();                                                                                                                                                                                                   // Constructor
        ProjectionInterval(const BoundingVolume* const boudingVolume, double min, double max, ExtremeType minType, ExtremeType maxType, std::vector<Vector3D> minProjectedPoints, std::vector<Vector3D> maxProjectedPoints);    // Constructor
        ~ProjectionInterval();                                                                                                                                                                                                  // Destructor

        BoundingVolume* getBoundingVolumePointer() const;       // Return the pointer on the bounding volume
        double getMin() const;                                  // Return the minimum of the interval
        double getMax() const;                                  // Return the maximum of the interval
        ExtremeType getMinType() const;                         // Return the type of the minimum extreme
        ExtremeType getMaxType() const;                         // Return the type of the maximum extreme
        std::vector<Vector3D> getMinProjectedPoints() const;    // Return the projected points onto the minimum extreme
        std::vector<Vector3D> getMaxProjectedPoints() const;    // Return the projected points onto the maximum extreme
};

// Return the pointer on the bounding volume
inline BoundingVolume* ProjectionInterval::getBoundingVolumePointer() const {
    return boundingVolume;
}

// Return the minimum of the interval
inline double ProjectionInterval::getMin() const {
    return min;
}

// Return the maximum of the interval
inline double ProjectionInterval::getMax() const {
    return max;
}

// Return the type of the minimum extreme
inline ExtremeType ProjectionInterval::getMinType() const {
    return minType;
}

// Return the type of the maximum extreme
inline ExtremeType ProjectionInterval::getMaxType() const {
    return maxType;
}

// Return the projected points onto the minimum extreme
inline std::vector<Vector3D> ProjectionInterval::getMinProjectedPoints() const {
    return minProjectedPoints;
}

// Return the projected points onto the maximum extreme
inline std::vector<Vector3D> ProjectionInterval::getMaxProjectedPoints() const {
    return maxProjectedPoints;
}

} // End of the ReactPhysics3D namespace

#endif
