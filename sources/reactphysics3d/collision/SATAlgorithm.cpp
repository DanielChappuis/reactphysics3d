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
#include "SATAlgorithm.h"
#include "../body/OBB.h"
#include "../body/RigidBody.h"
#include "../constraint/Contact.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
SATAlgorithm::SATAlgorithm() {

}

// Destructor
SATAlgorithm::~SATAlgorithm() {

}

// Return true and compute a contact info if the two bounding volume collide.
// The method returns false if there is no collision between the two bounding volumes.
bool SATAlgorithm::testCollision(const BoundingVolume* const boundingVolume1, const BoundingVolume* const boundingVolume2, ContactInfo*& contactInfo) {
    
    assert(boundingVolume1 != boundingVolume2);

    // If the two bounding volumes are OBB
    //const OBB* const obb1 = dynamic_cast<const OBB* const>(boundingVolume1);
    //const OBB* const obb2 = dynamic_cast<const OBB* const>(boundingVolume2);
    const OBB* const obb1 = dynamic_cast<const OBB* const>(boundingVolume1);
    const OBB* const obb2 = dynamic_cast<const OBB* const>(boundingVolume2);

    // If the two bounding volumes are OBB
    if (obb1 && obb2) {
        // Compute the collision test between two OBB
        return computeCollisionTest(obb1, obb2, contactInfo);
    }
    else {
        return false;
    }
}


// This method returns true and computes a contact info if the two OBB intersect.
// This method implements the separating algorithm between two OBBs. The goal of this method is to test if the
// two OBBs intersect or not. If they intersect we report a contact info and the method returns true. If
// they don't intersect, the method returns false. The separation axis that have to be tested for two
// OBB are the six face normals (3 for each OBB) and the nine vectors V = Ai x Bj where Ai is the ith face normal
// vector of OBB 1 and Bj is the jth face normal vector of OBB 2. We will use the notation Ai for the ith face
// normal of OBB 1 and Bj for the jth face normal of OBB 2.
bool SATAlgorithm::computeCollisionTest(const OBB* const obb1, const OBB* const obb2, ContactInfo*& contactInfo) const {


    double center;                              // Center of a projection interval
    double radius1;                             // Radius of projection interval [min1, max1]
    double radius2;                             // Radius of projection interval [min2, max2]
    double min1;                                // Minimum of interval 1
    double max1;                                // Maximum of interval 1
    double min2;                                // Minimm of interval 2
    double max2;                                // Maximum of interval 2
    Vector3D normal;                            // Contact normal (correspond to the separation axis with the smallest positive penetration depth)
                                                // The contact normal point out of OBB1 toward OBB2
    double minPenetrationDepth = DBL_MAX;       // Minimum penetration depth detected among all separated axis
    const double cutoff = 0.99;                 // Cutoff for cosine of angles between box axes
    bool existsParallelPair = false;            // True if there exists two face normals that are parallel.
                                                // This is used because if a parallel pair exists, it is sufficient
                                                // to test only the face normals of the OBBs for separation. Two nearly
                                                // parallel faces can lead to all face normal tests reporting no separation
                                                // along those directions. The cross product directions are tested next, but
                                                // Ai x Bj is nearly the zero vector and can cause a report that the two OBBs
                                                // are not intersecting when in fact they are.
    double c[3][3];                             // c[i][j] = DotProduct(obb1.Ai, obb2.Bj)
    double absC[3][3];                          // absC[i][j] = abs(DotProduct(obb1.Ai, obb2.Bj))
    double udc1[3];                             // DotProduct(obb1.Ai, obb2.center - obb1.center)
    double udc2[3];                             // DotProduct(obb2.Ai, obb2.center - obb1.center)
     
    Vector3D boxDistance = obb2->getCenter() - obb1->getCenter();   // Vector between the centers of the OBBs

    // Axis A0
    for (int i=0; i<3; ++i) {
        c[0][i] = obb1->getAxis(0).scalarProduct(obb2->getAxis(i));
        absC[0][i] = fabs(c[0][i]);
        if (absC[0][i] > cutoff) {
            existsParallelPair = true;
        }
    }
    udc1[0] = obb1->getAxis(0).scalarProduct(boxDistance);
    center = udc1[0];
    radius1 = obb1->getExtent(0);
    radius2 = obb2->getExtent(0)*absC[0][0] + obb2->getExtent(1)*absC[0][1] + obb2->getExtent(2) * absC[0][2];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    double penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(0), boxDistance);   // Compute the contact normal with the correct sign
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
    center = udc1[1];
    radius1 = obb1->getExtent(1);
    radius2 = obb2->getExtent(0)*absC[1][0] + obb2->getExtent(1)*absC[1][1] + obb2->getExtent(2) * absC[1][2];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(1), boxDistance);   // Compute the contact normal with the correct sign
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
    center = udc1[2];
    radius1 = obb1->getExtent(2);
    radius2 = obb2->getExtent(0)*absC[2][0] + obb2->getExtent(1)*absC[2][1] + obb2->getExtent(2)*absC[2][2];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(2), boxDistance);   // Compute the contact normal with the correct sign
    }

    // Axis B0
    udc2[0] = obb2->getAxis(0).scalarProduct(boxDistance);
    center = udc2[0];
    radius1 = obb1->getExtent(0)*absC[0][0] + obb1->getExtent(1)*absC[1][0] + obb1->getExtent(2) * absC[2][0];
    radius2 = obb2->getExtent(0);
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                         // Update the minimum penetration depth
        normal = computeContactNormal(obb2->getAxis(0), boxDistance);   // Compute the contact normal with the correct sign
    }

    // Axis B1
    udc2[1] = obb2->getAxis(1).scalarProduct(boxDistance);
    center = udc2[1];
    radius1 = obb1->getExtent(0)*absC[0][1] + obb1->getExtent(1)*absC[1][1] + obb1->getExtent(2) * absC[2][1];
    radius2 = obb2->getExtent(1);
    min1 = - radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                         // Update the minimum penetration depth
        normal = computeContactNormal(obb2->getAxis(1), boxDistance);   // Compute the contact normal with the correct sign
    }

    // Axis B2
    udc2[2] = obb2->getAxis(2).scalarProduct(boxDistance);
    center = udc2[2];
    radius1 = obb1->getExtent(0)*absC[0][2] + obb1->getExtent(1)*absC[1][2] + obb1->getExtent(2)*absC[2][2];
    radius2 = obb2->getExtent(2);
    min1 = - radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                         // Update the minimum penetration depth
        normal = computeContactNormal(obb2->getAxis(2), boxDistance);   // Compute the contact normal with the correct sign
    }

    // If there exists a parallel pair of face normals
    if (existsParallelPair) {
        // There exists a parallel pair of face normals and we have already checked all the face
        // normals for separation. Therefore the OBBs must intersect

        // Compute the contact info
        contactInfo = new ContactInfo(obb1, obb2, normal.getUnit(), minPenetrationDepth);
        
        return true;
    }

    // Axis A0 x B0
    center = udc1[2] * c[1][0] - udc1[1] * c[2][0];
    radius1 = obb1->getExtent(1) * absC[2][0] + obb1->getExtent(2) * absC[1][0];
    radius2 = obb2->getExtent(1) * absC[0][2] + obb2->getExtent(2) * absC[0][1];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(0).crossProduct(obb2->getAxis(0)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Axis A0 x B1
    center = udc1[2] * c[1][1] - udc1[1] * c[2][1];
    radius1 = obb1->getExtent(1) * absC[2][1] + obb1->getExtent(2) * absC[1][1];
    radius2 = obb2->getExtent(0) * absC[0][2] + obb2->getExtent(2) * absC[0][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(0).crossProduct(obb2->getAxis(1)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Axis A0 x B2
    center = udc1[2] * c[1][2] - udc1[1] * c[2][2];
    radius1 = obb1->getExtent(1) * absC[2][2] + obb1->getExtent(2) * absC[1][2];
    radius2 = obb2->getExtent(0) * absC[0][1] + obb2->getExtent(1) * absC[0][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(0).crossProduct(obb2->getAxis(2)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Axis A1 x B0
    center = udc1[0] * c[2][0] - udc1[2] * c[0][0];
    radius1 = obb1->getExtent(0) * absC[2][0] + obb1->getExtent(2) * absC[0][0];
    radius2 = obb2->getExtent(1) * absC[1][2] + obb2->getExtent(2) * absC[1][1];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(1).crossProduct(obb2->getAxis(0)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Axis A1 x B1
    center = udc1[0] * c[2][1] - udc1[2] * c[0][1];
    radius1 = obb1->getExtent(0) * absC[2][1] + obb1->getExtent(2) * absC[0][1];
    radius2 = obb2->getExtent(0) * absC[1][2] + obb2->getExtent(2) * absC[1][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(1).crossProduct(obb2->getAxis(1)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Axis A1 x B2
    center = udc1[0] * c[2][2] - udc1[2] * c[0][2];
    radius1 = obb1->getExtent(0) * absC[2][2] + obb1->getExtent(2) * absC[0][2];
    radius2 = obb2->getExtent(0) * absC[1][1] + obb2->getExtent(1) * absC[1][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(1).crossProduct(obb2->getAxis(2)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Axis A2 x B0
    center = udc1[1] * c[0][0] - udc1[0] * c[1][0];
    radius1 = obb1->getExtent(0) * absC[1][0] + obb1->getExtent(1) * absC[0][0];
    radius2 = obb2->getExtent(1) * absC[2][2] + obb2->getExtent(2) * absC[2][1];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(2).crossProduct(obb2->getAxis(0)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Axis A2 x B1
    center = udc1[1] * c[0][1] - udc1[0] * c[1][1];
    radius1 = obb1->getExtent(0) * absC[1][1] + obb1->getExtent(1) * absC[0][1];
    radius2 = obb2->getExtent(0) * absC[2][2] + obb2->getExtent(2) * absC[2][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(2).crossProduct(obb2->getAxis(1)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Axis A2 x B2
    center = udc1[1] * c[0][2] - udc1[0] * c[1][2];
    radius1 = obb1->getExtent(0) * absC[1][2] + obb1->getExtent(1) * absC[0][2];
    radius2 = obb2->getExtent(0) * absC[2][1] + obb2->getExtent(1) * absC[2][0];
    min1 = -radius1;
    max1 = radius1;
    min2 = center - radius2;
    max2 = center + radius2;
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(2).crossProduct(obb2->getAxis(2)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Compute the contact info
    contactInfo = new ContactInfo(obb1, obb2, normal.getUnit(), minPenetrationDepth);
    
    return true;
}

// This method computes and returns the penetration depth between two intervals. This method returns the computed
// penetration depth (note that it could return a negative penetration depth if the intervals are separated.
double SATAlgorithm::computePenetrationDepth(double min1, double max1, double min2, double max2) const {

    // Compute the length of both intervals
    double lengthInterval1 = max1 - min1;
    double lengthInterval2 = max2 - min2;

    // Compute the total length of both intervals
    double minExtreme = std::min(min1, min2);
    double maxExtreme = std::max(max1, max2);
    double lengthBothIntervals = maxExtreme - minExtreme;

    // Compute the current penetration depth
    return (lengthInterval1 + lengthInterval2) - lengthBothIntervals;
}
