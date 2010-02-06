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
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// TODO : Check and modify all comments on this file in order that
//        everything have to do with the new SAT algorithm

// Constructor
SATAlgorithm::SATAlgorithm() {

}

// Destructor
SATAlgorithm::~SATAlgorithm() {

}

// Return true and compute a collision contact if the two bounding volume collide.
// The method returns false if there is no collision between the two bounding volumes.
bool SATAlgorithm::testCollision(const BoundingVolume* const boundingVolume1, const BoundingVolume* const boundingVolume2, Contact** contact) {

    assert(boundingVolume1 != boundingVolume2);
    assert(*contact == 0);

    // If the two bounding volumes are OBB
    const OBB* const obb1 = dynamic_cast<const OBB* const>(boundingVolume1);
    const OBB* const obb2 = dynamic_cast<const OBB* const>(boundingVolume2);

    // If the two bounding volumes are OBB
    if (obb1 && obb2) {
        // Compute the collision test between two OBB
        return computeCollisionTest(obb1, obb2, contact);
    }
    else {
        return false;
    }
}


// This method returns true and computes a collision contact if the two OBB intersect.
// This method implements the separating algorithm between two OBB. The goal of this method is to test if the
// two OBBs intersect or not. If they intersect we report a collision contact and the method returns true. If
// they don't intersect, the method returns false. The separation axis that have to be tested for two
// OBB are the six face normals (3 for each OBB) and the nine vectors V = Ai x Bj where Ai is the ith face normal
// vector of OBB 1 and Bj is the jth face normal vector of OBB 2. We will use the notation Ai for the ith face
// normal of OBB 1 and Bj for the jth face normal of OBB 2.
bool SATAlgorithm::computeCollisionTest(const OBB* const obb1, const OBB* const obb2, Contact** contact) const {

    double center;                              // Center of a projection interval
    double radius1;                             // Radius of projection interval [min1, max1]
    double radius2;                             // Radius of projection interval [min2, max2]
    double min1;                                // Minimum of interval 1
    double max1;                                // Maximum of interval 1
    double min2;                                // Minimm of interval 2
    double max2;                                // Maximum of interval 2
    Vector3D normal;                            // Contact normal (correspond to the separation axis with the smallest positive penetration depth)
                                                // The contact normal point out of OBB1 toward OBB2
    bool side;                                  // True if the interval 1 is at the left of interval 2 if a collision occurs and false otherwise
    double minPenetrationDepth = DBL_MAX;       // Minimum penetration depth detected among all separated axis
    const double cutoff = 0.999999;             // Cutoff for cosine of angles between box axes
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
    bool sideTemp;              // TODO : Check if we really need the "side" variable (maybee, we can remove it)
    double penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
        minPenetrationDepth = penetrationDepth;                         // Update the minimum penetration depth
        normal = computeContactNormal(obb2->getAxis(2), boxDistance);   // Compute the contact normal with the correct sign
    }

    // If there exists a parallel pair of face normals
    if (existsParallelPair) {
        // There exists a parallel pair of face normals and we have already checked all the face
        // normals for separation. Therefore the OBBs must intersect

        // Compute the collision contact
        computeContact(obb1, obb2, normal.getUnit(), minPenetrationDepth, obb1->getExtremeVertices(normal), obb2->getExtremeVertices(normal.getOpposite()), contact);

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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
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
    penetrationDepth = computePenetrationDepth(min1, max1, min2, max2, sideTemp);
    if (penetrationDepth < 0) { // We have found a separation axis, therefore the two OBBs don't collide
        return false;
    }
    else if (penetrationDepth < minPenetrationDepth) {  // Interval 1 and 2 overlap with a smaller penetration depth on this axis
        side = sideTemp;
        minPenetrationDepth = penetrationDepth;                                                         // Update the minimum penetration depth
        normal = computeContactNormal(obb1->getAxis(2).crossProduct(obb2->getAxis(2)), boxDistance);    // Compute the contact normal with the correct sign
    }

    // Compute the collision contact
    computeContact(obb1, obb2, normal.getUnit(), minPenetrationDepth, obb1->getExtremeVertices(normal), obb2->getExtremeVertices(normal.getOpposite()), contact);

    return true;
}

// This method computes and returns the penetration depth between two intervals. This method returns the computed
// penetration depth (note that it could return a negative penetration depth if the intervals are separated. This
// method also find which interval is at the left of the other in order to know which extreme of interval 1 collides with
// which extreme of interval 2 if a collision occur.
double SATAlgorithm::computePenetrationDepth(double min1, double max1, double min2, double max2, bool& side) const {

    // Compute the length of both intervals
    double lengthInterval1 = max1 - min1;
    double lengthInterval2 = max2 - min2;

    // Compute the total length of both intervals
    double minExtreme = std::min(min1, min2);
    double maxExtreme = std::max(max1, max2);
    double lengthBothIntervals = maxExtreme - minExtreme;

    // Compute the current penetration depth
    double penetrationDepth = (lengthInterval1 + lengthInterval2) - lengthBothIntervals;

    // Find which interval is at the left of the other
    if (abs(max1-min2) <= abs(max2-min1)) {
        // Right of interval 1 collides with the left of interval 2
        side = true;
    }
    else {
        // Right of interval 2 collides with the left of interval 1
        side = false;
    }

    // Return the computed penetration depth
    return penetrationDepth;
}

// Compute a new collision contact between two OBBs
void SATAlgorithm::computeContact(const OBB* const obb1, const OBB* const obb2, const Vector3D normal, double penetrationDepth,
                                  const std::vector<Vector3D>& obb1ExtremePoints, const std::vector<Vector3D>& obb2ExtremePoints, Contact** contact) const {

    unsigned int nbVerticesExtremeOBB1 = obb1ExtremePoints.size();
    unsigned int nbVerticesExtremeOBB2 = obb2ExtremePoints.size();
    assert(nbVerticesExtremeOBB1==1 || nbVerticesExtremeOBB1==2 || nbVerticesExtremeOBB1==4);
    assert(nbVerticesExtremeOBB2==1 || nbVerticesExtremeOBB2==2 || nbVerticesExtremeOBB2==4);
    assert(approxEqual(normal.length(), 1.0));

    // If it's a Vertex-Something contact
    if (nbVerticesExtremeOBB1 == 1) {
        // Create a new contact
        *contact = new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, obb1ExtremePoints);
    }
    else if(nbVerticesExtremeOBB2 == 1) {  // If its a Vertex-Something contact
        // Create a new contact
        *contact = new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, obb2ExtremePoints);
    }
    else if (nbVerticesExtremeOBB1 == 2 && nbVerticesExtremeOBB2 == 2) {    // If it's an edge-edge contact
        // Compute the two vectors of the segment lines
        Vector3D d1 = obb1ExtremePoints[1] - obb1ExtremePoints[0];
        Vector3D d2 = obb2ExtremePoints[1] - obb2ExtremePoints[0];

        double alpha, beta;
        std::vector<Vector3D> contactSet;

        // If the two edges are parallel
        if (d1.isParallelWith(d2)) {
            Vector3D contactPointA;
            Vector3D contactPointB;

            // Compute the intersection between the two edges
            computeParallelSegmentsIntersection(obb1ExtremePoints[0], obb1ExtremePoints[1], obb2ExtremePoints[0], obb2ExtremePoints[1],
                                                contactPointA, contactPointB);

            // Add the two contact points in the contact set
            contactSet.push_back(contactPointA);
            contactSet.push_back(contactPointB);
        }
        else {  // If the two edges are not parallel
            // Compute the closest two points between the two line segments
            closestPointsBetweenTwoLines(obb1ExtremePoints[0], d1, obb2ExtremePoints[0], d2, &alpha, &beta);
            Vector3D pointA = obb1ExtremePoints[0] + d1 * alpha;
            Vector3D pointB = obb2ExtremePoints[0] + d2 * beta;

            // Compute the contact point as halfway between the 2 closest points
            Vector3D contactPoint = 0.5 * (pointA + pointB);

            // Add the contact point into the contact set
            contactSet.push_back(contactPoint);
        }

        // Create a new contact
        *contact = new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, contactSet);
    }
    else if(nbVerticesExtremeOBB1 == 2 && nbVerticesExtremeOBB2 == 4) {     // If it's an edge-face contact
        // Compute the projection of the edge of OBB1 onto the same plane of the face of OBB2
        std::vector<Vector3D> edge = projectPointsOntoPlane(obb1ExtremePoints, obb2ExtremePoints[0], normal);

        // Clip the edge of OBB1 using the face of OBB2
        std::vector<Vector3D> clippedEdge = clipSegmentWithRectangleInPlane(edge, obb2ExtremePoints);

        // Move the clipped edge halway between the edge of OBB1 and the face of OBB2
        clippedEdge = movePoints(clippedEdge, penetrationDepth/2.0 * normal.getOpposite());

        // Create a new contact
        *contact = new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, clippedEdge);
    }
    else if(nbVerticesExtremeOBB1 == 4 && nbVerticesExtremeOBB2 == 2) {     // If it's an edge-face contact
        // Compute the projection of the edge of OBB2 onto the same plane of the face of OBB1
        std::vector<Vector3D> edge = projectPointsOntoPlane(obb2ExtremePoints, obb1ExtremePoints[0], normal);

        // Clip the edge of OBB2 using the face of OBB1
        std::vector<Vector3D> clippedEdge = clipSegmentWithRectangleInPlane(edge, obb1ExtremePoints);

        // Move the clipped edge halfway between the face of OBB1 and the edge of OBB2
        clippedEdge = movePoints(clippedEdge, penetrationDepth/2.0 * normal);

        // Create a new contact
        *contact = new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, clippedEdge);
    }
    else {      // If it's a face-face contact
        // Compute the projection of the face vertices of OBB2 onto the plane of the face of OBB1
        std::vector<Vector3D> faceOBB2 = projectPointsOntoPlane(obb2ExtremePoints, obb1ExtremePoints[0], normal);

        // Clip the face of OBB2 using the face of OBB1
        std::vector<Vector3D> clippedFace = clipPolygonWithRectangleInPlane(faceOBB2, obb1ExtremePoints);

        // Move the clipped face halfway between the face of OBB1 and the face of OBB2
        clippedFace = movePoints(clippedFace, penetrationDepth/2.0 * normal);

        // Create a new contact
        *contact = new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, clippedFace);
    }

    assert(*contact != 0);
}
