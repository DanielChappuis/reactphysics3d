
/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2011 Daniel Chappuis                                            *
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

// Libraries
#include "GJKAlgorithm.h"
#include "Simplex.h"
#include "../../constraint/Contact.h"
#include "../../constants.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cassert>


// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// TODO : Check that allocated memory is correctly deleted

// Constructor
GJKAlgorithm::GJKAlgorithm() {
    
}

// Destructor
GJKAlgorithm::~GJKAlgorithm() {

}

// Return true and compute a contact info if the two bounding volume collide.
// This method implements the Hybrid Technique for computing the penetration depth by
// running the GJK algorithm on original objects (without margin).
// If the objects don't intersect, this method returns false. If they intersect
// only in the margins, the method compute the penetration depth and contact points
// (of enlarged objects). If the original objects (without margin) intersect, we
// call the computePenetrationDepthForEnlargedObjects() method that run the GJK
// algorithm on the enlarged object to obtain a simplex polytope that contains the
// origin, they we give that simplex polytope to the EPA algorithm which will compute
// the correct penetration depth and contact points between the enlarged objects.
bool GJKAlgorithm::testCollision(const NarrowBoundingVolume* const boundingVolume1, const Transform& transform1,
                                 const NarrowBoundingVolume* const boundingVolume2, const Transform& transform2,
                                 ContactInfo*& contactInfo) {
    Vector3D suppA;             // Support point of object A
    Vector3D suppB;             // Support point of object B
    Vector3D w;                 // Support point of Minkowski difference A-B
    Vector3D pA;                // Closest point of object A
    Vector3D pB;                // Closest point of object B
    double vDotw;
    double prevDistSquare;

    // Transform a point from body space of shape 2 to body space of shape 1 (the GJK algorithm is done in body space of shape 1)
    Transform shape2ToShape1 = transform1.inverse() * transform2;

    // Matrix that transform a direction from body space of shape 1 into body space of shape 2
    Matrix3x3 rotateToShape2 = transform2.getOrientation().getTranspose() * transform1.getOrientation();

    assert(boundingVolume1 != boundingVolume2);

    // Initialize the margin (sum of margins of both objects)
    double margin = 2 * OBJECT_MARGIN;
    double marginSquare = margin * margin;
    assert(margin > 0.0);

    // Create a simplex set
    Simplex simplex;

    // Get the last point V (last separating axis)
    // TODO : Implement frame coherence. For each pair of body, store
    //        the last separating axis and use it to initialize the v vector
    Vector3D v(0.0, 1.0, 0.0);

    // Initialize the upper bound for the square distance
    double distSquare = DBL_MAX;
    
    do {
        // Compute the support points for original objects (without margins) A and B
        suppA = boundingVolume1->getSupportPoint(v.getOpposite());
        suppB = shape2ToShape1 * boundingVolume2->getSupportPoint(rotateToShape2 * v);

        // Compute the support point for the Minkowski difference A-B
        w = suppA - suppB;
        
        vDotw = v.dot(w);
        
        // If the enlarge objects (with margins) do not intersect
        if (vDotw > 0.0 && vDotw * vDotw > distSquare * marginSquare) {
            // No intersection, we return false
            return false;
        }

        // If the objects intersect only in the margins
        if (simplex.isPointInSimplex(w) || distSquare - vDotw <= distSquare * REL_ERROR_SQUARE) {
            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            double dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = transform1 * (pA - (OBJECT_MARGIN / dist) * v);
            pB = transform1 * (pB + (OBJECT_MARGIN / dist) * v);

            // Compute the contact info
            Vector3D normal = transform1.getOrientation() * v.getOpposite().getUnit();
            double penetrationDepth = margin - dist;
            contactInfo = new ContactInfo(boundingVolume1->getBodyPointer(), boundingVolume2->getBodyPointer(),
                                          normal, penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        // Add the new support point to the simplex
        simplex.addPoint(w, suppA, suppB);

        // If the simplex is affinely dependent
        if (simplex.isAffinelyDependent()) {
            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            double dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = transform1 * (pA - (OBJECT_MARGIN / dist) * v);
            pB = transform1 * (pB + (OBJECT_MARGIN / dist) * v);

            // Compute the contact info
            Vector3D normal = transform1.getOrientation() * v.getOpposite().getUnit();
            double penetrationDepth = margin - dist;
            contactInfo = new ContactInfo(boundingVolume1->getBodyPointer(), boundingVolume2->getBodyPointer(),
                                          normal, penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        // Compute the point of the simplex closest to the origin
        // If the computation of the closest point fail
        if (!simplex.computeClosestPoint(v)) {
            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            double dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = transform1 * (pA - (OBJECT_MARGIN / dist) * v);
            pB = transform1 * (pB + (OBJECT_MARGIN / dist) * v);

            // Compute the contact info
            Vector3D normal = transform1.getOrientation() * v.getOpposite().getUnit();
            double penetrationDepth = margin - dist;
            contactInfo = new ContactInfo(boundingVolume1->getBodyPointer(), boundingVolume2->getBodyPointer(),
                                          normal, penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        // Store and update the squared distance of the closest point
        prevDistSquare = distSquare;
        distSquare = v.dot(v);

        // If the distance to the closest point doesn't improve a lot
        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {
            simplex.backupClosestPointInSimplex(v);
            
            // Get the new squared distance
            distSquare = v.dot(v);

            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            double dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = transform1 * (pA - (OBJECT_MARGIN / dist) * v);
            pB = transform1 * (pB + (OBJECT_MARGIN / dist) * v);

            // Compute the contact info
            Vector3D normal = transform1.getOrientation() * v.getOpposite().getUnit();
            double penetrationDepth = margin - dist;
            contactInfo = new ContactInfo(boundingVolume1->getBodyPointer(), boundingVolume2->getBodyPointer(),
                                          normal, penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        double test = simplex.getMaxLengthSquareOfAPoint(); // TODO : Remove this
        test = 4.5;
        
    } while(!simplex.isFull() && distSquare > MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());

    // The objects (without margins) intersect. Therefore, we run the GJK algorithm again but on the
    // enlarged objects to compute a simplex polytope that contains the origin. Then, we give that simplex
    // polytope to the EPA algorithm to compute the correct penetration depth and contact points between
    // the enlarged objects.
    return computePenetrationDepthForEnlargedObjects(boundingVolume1, transform1, boundingVolume2, transform2, contactInfo, v);
}

// This method runs the GJK algorithm on the two enlarged objects (with margin)
// to compute a simplex polytope that contains the origin. The two objects are
// assumed to intersect in the original objects (without margin). Therefore such
// a polytope must exist. Then, we give that polytope to the EPA algorithm to
// compute the correct penetration depth and contact points of the enlarged objects.
bool GJKAlgorithm::computePenetrationDepthForEnlargedObjects(const NarrowBoundingVolume* const boundingVolume1, const Transform& transform1,
                                                             const NarrowBoundingVolume* const boundingVolume2, const Transform& transform2,
                                                             ContactInfo*& contactInfo, Vector3D& v) {
    Simplex simplex;
    Vector3D suppA;
    Vector3D suppB;
    Vector3D w;
    double vDotw;
    double distSquare = DBL_MAX;
    double prevDistSquare;

    // Transform a point from body space of shape 2 to body space of shape 1 (the GJK algorithm is done in body space of shape 1)
    Transform shape2ToShape1 = transform1.inverse() * transform2;

    // Matrix that transform a direction from body space of shape 1 into body space of shape 2
    Matrix3x3 rotateToShape2 = transform2.getOrientation().getTranspose() * transform1.getOrientation();
    
    do {
        // Compute the support points for the enlarged object A and B
        suppA = boundingVolume1->getSupportPoint(v.getOpposite(), OBJECT_MARGIN);
        suppB = shape2ToShape1 * boundingVolume2->getSupportPoint(rotateToShape2 * v, OBJECT_MARGIN);

        // Compute the support point for the Minkowski difference A-B
        w = suppA - suppB;

        vDotw = v.dot(w);

        // If the enlarge objects do not intersect
        if (vDotw > 0.0) {
            // No intersection, we return false
            return false;
        }

        // Add the new support point to the simplex
        simplex.addPoint(w, suppA, suppB);

        if (simplex.isAffinelyDependent()) {
            return false;
        }

        if (!simplex.computeClosestPoint(v)) {
            return false;
        }

        // Store and update the square distance
        prevDistSquare = distSquare;
        distSquare = v.dot(v);

        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {
            return false;
        }

    } while(!simplex.isFull() && distSquare > MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());

    // Give the simplex computed with GJK algorithm to the EPA algorithm which will compute the correct
    // penetration depth and contact points between the two enlarged objects
    return algoEPA.computePenetrationDepthAndContactPoints(simplex, boundingVolume1, transform1,
                                                           boundingVolume2, transform2, v, contactInfo);
}
