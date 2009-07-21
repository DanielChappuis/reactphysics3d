/***************************************************************************
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

// Libraries
#include "SeparatingAxisOBB.h"
#include "../body/OBB.h"
#include "../constraint/Contact.h"
#include <cfloat>
#include <iostream> // TODO : Delete this
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
SeparatingAxisOBB::SeparatingAxisOBB() {

}

// Destructor
SeparatingAxisOBB::~SeparatingAxisOBB() {

}

// Return true and compute a collision contact if the two bounding volume collide.
// The method returns false if there is no collision between the two bounding volumes.
bool SeparatingAxisOBB::testCollision(const BoundingVolume* const boundingVolume1, const BoundingVolume* const boundingVolume2, Contact** contact,
                                      const Vector3D& velocity1, const Vector3D& velocity2, const Time& timeMax, Time& timeFirst, Time& timeLast) {
    assert(boundingVolume1 != boundingVolume2);

    // If the two bounding volumes are OBB
    const OBB* const obb1 = dynamic_cast<const OBB* const>(boundingVolume1);
    const OBB* const obb2 = dynamic_cast<const OBB* const>(boundingVolume2);

    // If the two bounding volumes are OBB
    if (obb1 && obb2) {
        // Compute the collision test between two OBB
        return computeCollisionTest(obb1, obb2, contact, velocity1, velocity2, timeMax, timeFirst, timeLast);
    }
    else {
        return false;
    }
}

// Return true and compute a collision contact if the two OBB collide.
// This method implements the separating algorithm between two OBB. The goal of this method is to compute the
// time (in the interval [0, timeMax] at wich the two bodies will collide if they will collide. If they will
// collide we report a collision contact. "velocity1" and "velocity2" are the velocity vectors of the two bodies.
// If they collide, timeFirst will contain the first collision time of the two bodies and timeLast will contain
// the time when the two bodies separate after the collision. The separation axis that have to be tested for two
// OBB are the six face normals (3 for each OBB) and the nine vectors V = Ai x Bj where Ai is the ith face normal
// vector of OBB 1 and Bj is the jth face normal vector of OBB 2. We will use the notation Ai for the ith face
// normal of OBB 1 and Bj for the jth face normal of OBB 2.
bool SeparatingAxisOBB::computeCollisionTest(const OBB* const obb1, const OBB* const obb2, Contact** contact,
                                            const Vector3D& velocity1, const Vector3D& velocity2, const Time& timeMax,
                                            Time& timeFirst, Time& timeLast) {

    double center;                      // Center
    double speed;                       // Relavtive speed of the projection intervals (dotProduct(SeparatingAxis, deltaVelocity))
    double radius1;                     // Radius of projection interval [min1, max1]
    double radius2;                     // Radius of projection interval [min2, max2]
    double min1;                        // Minimum of interval 1
    double max1;                        // Maximum of interval 1
    double min2;                        // Minimm of interval 2
    double max2;                        // Maximum of interval 2
    const double cutoff = 0.999999;     // Cutoff for cosine of angles between box axes
    bool existsParallelPair = false;    // True if there exists two face normals that are parallel.
                                        // This is used because if a parallel pair exists, it is sufficient
                                        // to test only the face normals of the OBBs for separation. Two nearly
                                        // parallel faces can lead to all face normal tests reporting no separation
                                        // along those directions. The cross product directions are tested next, but
                                        // Ai x Bj is nearly the zero vector and can cause a report that the two OBBs
                                        // are not intersecting when in fact they are.
    double c[3][3];                     // c[i][j] = DotProduct(obb1.Ai, obb2.Bj)
    double absC[3][3];                  // absC[i][j] = abs(DotProduct(obb1.Ai, obb2.Bj))
    double udc1[3];                     // DotProduct(obb1.Ai, obb2.center - obb1.center)
    double udv1[3];                     // DotProduct(obb1.Ai, velocity2 - velocity1)
    double udc2[3];                     // DotProduct(obb2.Ai, obb2.center - obb1.center)
    double udv2[3];                     // DotProduct(obb2.Ai, velocity2 - velocity1)

    Vector3D deltaVelocity = velocity2-velocity1;                   // Difference of box center velocities
    Vector3D boxDistance = obb2->getCenter() - obb1->getCenter();   // Distance between the centers of the OBBs
    timeFirst.setValue(0.0);                                        // timeFirst = 0
    timeLast.setValue(DBL_MAX);                                     // timeLast = infinity

    // Axis A0
    for (int i=0; i<3; ++i) {
        c[0][i] = obb1->getAxis(0).scalarProduct(obb2->getAxis(i));
        absC[0][i] = fabs(c[0][i]);
        if (absC[0][i] > cutoff) {
            existsParallelPair = true;
        }
    }
    udc1[0] = obb1->getAxis(0).scalarProduct(boxDistance);
    udv1[0] = obb1->getAxis(0).scalarProduct(deltaVelocity);
    center = udc1[0];
    speed = udv1[0];
    radius1 = obb1->getExtent(0);
    radius2 = obb2->getExtent(0)*absC[0][0] + obb2->getExtent(1)*absC[0][1] + obb2->getExtent(2) * absC[0][2];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    std::cout << "Speed : " << speed << std::endl;
    std::cout << "min1 : " << min1 << std::endl;
    std::cout << "max1 : " << max1 << std::endl;
    std::cout << "min2 : " << min2 << std::endl;
    std::cout << "max2 : " << max2 << std::endl;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A1
    for (int i=0; i<3; ++i) {
        c[1][i] = obb1->getAxis(1).scalarProduct(obb2->getAxis(i));
        absC[1][i] = fabs(c[1][i]);
        if (absC[1][i] > cutoff) {
            existsParallelPair = true;
        }
    }
    udc1[1] = obb1->getAxis(1).scalarProduct(boxDistance);
    udv1[1] = obb1->getAxis(1).scalarProduct(deltaVelocity);
    center = udc1[1];
    std::cout << "Axis - X : " << obb1->getAxis(1).getX() << "Y : " << obb1->getAxis(1).getY() << "Z : " << obb1->getAxis(1).getZ() << std::endl;
    std::cout << "Distance - X : " << boxDistance.getX() << "Y : " << boxDistance.getY() << "Z : " << boxDistance.getZ() << std::endl;
    std::cout << "Center : " << center << std::endl;
    speed = udv1[1];
    radius1 = obb1->getExtent(1);
    radius2 = obb2->getExtent(0)*absC[1][0] + obb2->getExtent(1)*absC[1][1] + obb2->getExtent(2) * absC[1][2];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    std::cout << "Speed : " << speed << std::endl;
    std::cout << "min1 : " << min1 << std::endl;
    std::cout << "max1 : " << max1 << std::endl;
    std::cout << "min2 : " << min2 << std::endl;
    std::cout << "max2 : " << max2 << std::endl;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A2
    for (int i=0; i<3; ++i) {
        c[2][i] = obb1->getAxis(2).scalarProduct(obb2->getAxis(i));
        absC[2][i] = fabs(c[2][i]);
        if (absC[2][i] > cutoff) {
            existsParallelPair = true;
        }
    }
    udc1[2] = obb1->getAxis(2).scalarProduct(boxDistance);
    udv1[2] = obb1->getAxis(2).scalarProduct(deltaVelocity);
    center = udc1[2];
    speed = udv1[2];
    radius1 = obb1->getExtent(2);
    radius2 = obb2->getExtent(0)*absC[2][0] + obb2->getExtent(1)*absC[2][1] + obb2->getExtent(2) * absC[2][2];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    std::cout << "Speed : " << speed << std::endl;
    std::cout << "min1 : " << min1 << std::endl;
    std::cout << "max1 : " << max1 << std::endl;
    std::cout << "min2 : " << min2 << std::endl;
    std::cout << "max2 : " << max2 << std::endl;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis B0
    udc2[0] = obb2->getAxis(0).scalarProduct(boxDistance);
    udv2[0] = obb2->getAxis(0).scalarProduct(deltaVelocity);
    center = udc2[0];
    speed = udv2[0];
    radius1 = obb1->getExtent(0)*absC[0][0] + obb1->getExtent(1)*absC[1][0] + obb1->getExtent(2) * absC[1][0];
    radius2 = obb2->getExtent(0);
    min1 = center - radius1;
    max1 = center + radius1;
    min2 = -radius2;
    max2 = radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis B1
    udc2[1] = obb2->getAxis(1).scalarProduct(boxDistance);
    udv2[1] = obb2->getAxis(1).scalarProduct(deltaVelocity);
    center = udc2[1];
    speed = udv2[1];
    radius1 = obb1->getExtent(0)*absC[0][1] + obb1->getExtent(1)*absC[1][1] + obb1->getExtent(2) * absC[1][1];
    radius2 = obb2->getExtent(1);
    min1 = center - radius1;
    max1 = center + radius1;
    min2 = -radius2;
    max2 = radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis B2
    udc2[2] = obb2->getAxis(2).scalarProduct(boxDistance);
    udv2[2] = obb2->getAxis(2).scalarProduct(deltaVelocity);
    center = udc2[2];
    speed = udv2[2];
    radius1 = obb1->getExtent(0)*absC[0][2] + obb1->getExtent(1)*absC[1][2] + obb1->getExtent(2) * absC[1][2];
    radius2 = obb2->getExtent(2);
    min1 = center - radius1;
    max1 = center + radius1;
    min2 = -radius2;
    max2 = radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // If there exists a parallel pair of face normals
    if (existsParallelPair) {
        // There exists a parallel pair of face normals and we have already checked all the face
        // normals for separation. Therefore the OBBs must intersect

        // TODO : Delete this
        (*contact) = new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), Vector3D(1,0,0));
        std::cout << "Contact : " << contact << std::endl;

        return true;
    }

    // Axis A0 x B0
    center = udc1[2] * c[1][0] - udc1[1] * c[2][0];
    speed = udv1[2] * c[1][0] - udv1[1] * c[2][0];
    radius1 = obb1->getExtent(1) * absC[2][0] + obb1->getExtent(2) * absC[1][0];
    radius2 = obb2->getExtent(1) * absC[0][2] + obb2->getExtent(2) * absC[0][1];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A0 x B1
    center = udc1[2] * c[1][1] - udc1[1] * c[2][1];
    speed = udv1[2] * c[1][1] - udv1[1] * c[2][1];
    radius1 = obb1->getExtent(1) * absC[2][1] + obb1->getExtent(2) * absC[1][1];
    radius2 = obb2->getExtent(0) * absC[0][2] + obb2->getExtent(2) * absC[0][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A0 x B2
    center = udc1[2] * c[1][2] - udc1[1] * c[2][2];
    speed = udv1[2] * c[1][2] - udv1[1] * c[2][2];
    radius1 = obb1->getExtent(1) * absC[2][2] + obb1->getExtent(2) * absC[1][2];
    radius2 = obb2->getExtent(0) * absC[0][1] + obb2->getExtent(1) * absC[0][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A1 x B0
    center = udc1[0] * c[2][0] - udc1[2] * c[0][0];
    speed = udv1[0] * c[2][0] - udv1[2] * c[0][0];
    radius1 = obb1->getExtent(0) * absC[2][0] + obb1->getExtent(2) * absC[0][0];
    radius2 = obb2->getExtent(1) * absC[1][2] + obb2->getExtent(2) * absC[1][1];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A1 x B1
    center = udc1[0] * c[2][1] - udc1[2] * c[0][1];
    speed = udv1[0] * c[2][1] - udv1[2] * c[0][1];
    radius1 = obb1->getExtent(0) * absC[2][1] + obb1->getExtent(2) * absC[0][1];
    radius2 = obb2->getExtent(0) * absC[1][2] + obb2->getExtent(2) * absC[1][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A1 x B2
    center = udc1[0] * c[2][2] - udc1[2] * c[0][2];
    speed = udv1[0] * c[2][2] - udv1[2] * c[0][2];
    radius1 = obb1->getExtent(0) * absC[2][2] + obb1->getExtent(2) * absC[0][2];
    radius2 = obb2->getExtent(0) * absC[1][1] + obb2->getExtent(1) * absC[1][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A2 x B0
    center = udc1[1] * c[0][0] - udc1[0] * c[1][0];
    speed = udv1[1] * c[0][0] - udv1[0] * c[1][0];
    radius1 = obb1->getExtent(0) * absC[1][0] + obb1->getExtent(1) * absC[0][0];
    radius2 = obb2->getExtent(1) * absC[2][2] + obb2->getExtent(2) * absC[2][1];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A2 x B1
    center = udc1[1] * c[0][1] - udc1[0] * c[1][1];
    speed = udv1[1] * c[0][1] - udv1[0] * c[1][1];
    radius1 = obb1->getExtent(0) * absC[1][1] + obb1->getExtent(1) * absC[0][1];
    radius2 = obb2->getExtent(0) * absC[2][2] + obb2->getExtent(2) * absC[2][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // Axis A2 x B2
    center = udc1[1] * c[0][2] - udc1[0] * c[1][2];
    speed = udv1[1] * c[0][2] - udv1[0] * c[1][2];
    radius1 = obb1->getExtent(0) * absC[1][2] + obb1->getExtent(1) * absC[0][2];
    radius2 = obb2->getExtent(0) * absC[2][1] + obb2->getExtent(1) * absC[2][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    if(!computeIntervalsIntersectionTime(timeMax, speed, min1, max1, min2, max2, timeFirst, timeLast)) {
        // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }

    // TODO : Delete this
    (*contact) = new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), Vector3D(1,0,0));
    std::cout << "Contact2 : " << contact << std::endl;

    // We have found no separation axis, therefore the two OBBs must collide
    return true;
}

// This method computes the intersection time of two projection intervals.
// This method takes two projection intervals [min1, max1] and [min2, max2] and computes (if the
// two intervals intersect) in the time interval [0, timeMax] the time timeFirst where the two bodies
// enter in collision and the time timeLast where the two bodies separate themself from the collision.
// We consider that the interval 2 move at the speed "speed" and the interval 1 don't move.
// The method returns true if the two projection intervals intersect and false if they move appart.
// This method will be called for each separation axis.
bool SeparatingAxisOBB::computeIntervalsIntersectionTime(const Time& timeMax, double speed, double min1, double max1,
                                                         double min2, double max2, Time& timeFirst, Time& timeLast) {
    double speedInverse = 1.0/speed;
    double t;

    // If the interval [min0, max0] is on right of interval [min1, max1]
    if (max2 < min1) {
        // If the two intervals move apart they will not intersect
        if (speed <= 0) {
            return false;
        }

        // Compute the time t when the two intervals enter in contact
        t = (min1-max2) * speedInverse;

        // If we found a later collision time, we update the first collision time
        if (t > timeFirst.getValue()) {
            timeFirst.setValue(t);
        }

        // If the first collision time is outside of the time interval [0, timeMax]
        if(timeFirst.getValue() > timeMax.getValue()) {
            return false;
        }

        // Compute the time t when the two intervals separate from a contact
        t = (max1 - min2) * speedInverse;

        // If we found a earlier separated collision time, we update the last collision time
        if (t < timeLast.getValue()) {

                timeLast.setValue(t);
        }

        // If the first collision time occurs after the last collision time
        if (timeFirst.getValue() > timeLast.getValue()) {
            return false;
        }
    }
    else if (max1 < min2) {      // If the interval [min0, max0] is on left of interval [min1, max1]
        // If the two intervals move apart they will not intersect
        if (speed >= 0) {
            return false;
        }

        // Compute the time t when the two intervals enter in contact
        t = (max1 - min2) * speedInverse;

        // If we found a later collision time
        if (t > timeFirst.getValue()) {
            timeFirst.setValue(t);
        }

        // If the first collision time is outside of the time interval [0, timeMax]
        if(timeFirst.getValue() > timeMax.getValue()) {
            return false;
        }

        // Compute the time t when the two intervals separate from a contact
        t = (min1 - max2) * speedInverse;

        // If we found a earlier separated collision time
        if (t < timeLast.getValue()) {
                timeLast.setValue(t);
        }

        // If the first collision time occurs after the last collision time
        if (timeFirst.getValue() > timeLast.getValue()) {
            return false;
        }
    }
    else {  // If the two intervals overlap
        if (speed > 0) {
            // Compute the time t when the two intervals separate from a contact
            t = (max1 - min2) * speedInverse;

            // If we found a earlier separated collision time
            if (t < timeLast.getValue()) {
                timeLast.setValue(t);
            }

            // If the first collision time occurs after the last collision time
            if (timeFirst.getValue() > timeLast.getValue()) {
                return false;
            }
        }
        else if (speed < 0) {
            // Compute the time t when the two intervals separate from a contact
            t = (min1 - max2) * speedInverse;

            // If we found a earlier separated collision time
            if (t < timeLast.getValue()) {
                timeLast.setValue(t);
            }

            // If the first collision time occurs after the last collision time
            if (timeFirst.getValue() > timeLast.getValue()) {
                return false;
            }
        }
    }

    return true;
}
