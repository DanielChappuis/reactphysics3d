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
#include "CollisionDetection.h"
#include "NoBroadPhaseAlgorithm.h"
#include "SATAlgorithm.h"
#include "../body/Body.h"
#include "../body/OBB.h"
#include "../body/RigidBody.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionDetection::CollisionDetection(PhysicsWorld* world) {
    this->world = world;
    
    // Construct the broad-phase algorithm that will be used (Separating axis with AABB)
    broadPhaseAlgorithm = new NoBroadPhaseAlgorithm();

    // Construct the narrow-phase algorithm that will be used (Separating axis algorithm)
    narrowPhaseAlgorithm = new SATAlgorithm();
}

// Destructor
CollisionDetection::~CollisionDetection() {

}

// Compute the collision detection
bool CollisionDetection::computeCollisionDetection() {

    world->removeAllContactConstraints();
    possibleCollisionPairs.clear();
    contactInfos.clear();

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Compute the narrow-phase collision detection
    computeNarrowPhase();

    // Compute all the new contacts
    computeAllContacts();

    // Return true if at least one contact has been found
    return (contactInfos.size() > 0);
}

// Compute the broad-phase collision detection
void CollisionDetection::computeBroadPhase() {

    // For each pair of bodies in the physics world
    for(std::vector<Body*>::const_iterator it1 = world->getBodiesBeginIterator(); it1 != world->getBodiesEndIterator(); ++it1) {
        for(std::vector<Body*>::const_iterator it2 = it1; it2 != world->getBodiesEndIterator(); ++it2) {
            
            RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(*it1);
            RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(*it2);

            // If both bodies are RigidBody and are different and if both have collision activated
            if(rigidBody1 && rigidBody2 && rigidBody1 != rigidBody2
               && rigidBody1->getIsCollisionEnabled() && rigidBody2->getIsCollisionEnabled()) {
                // Get the oriented bounding boxes of the two bodies
                const OBB* obb1 = rigidBody1->getOBB();
                const OBB* obb2 = rigidBody2->getOBB();

                // Use the broad-phase algorithm to decide if the two bodies can collide
                if(broadPhaseAlgorithm->testCollisionPair(obb1, obb2)) {
                    // If the broad-phase thinks that the two bodies collide, we add the in the possible collision pair list
                    possibleCollisionPairs.push_back(std::pair<const OBB*, const OBB*>(obb1, obb2));
                }
            }
        }
    }
}

// Compute the narrow-phase collision detection
void CollisionDetection::computeNarrowPhase() {
    // For each possible collision pair of bodies
    for (unsigned int i=0; i<possibleCollisionPairs.size(); i++) {
        ContactInfo* contactInfo = 0;

        // Use the narrow-phase collision detection algorithm to check if the really are a contact
        if (narrowPhaseAlgorithm->testCollision(possibleCollisionPairs.at(i).first, possibleCollisionPairs.at(i).second, contactInfo)) {
            assert(contactInfo != 0);

            // Add the contact info the current list of collision informations
            contactInfos.push_back(contactInfo);
        }
    }
}

// Compute all the contacts from the contact info list
void CollisionDetection::computeAllContacts() {
    // For each possible contact info (computed during narrow-phase collision detection)
    for (unsigned int i=0; i<contactInfos.size(); i++) {
        ContactInfo* contactInfo = contactInfos.at(i);
        assert(contactInfo != 0);
        
        // Compute one or several new contacts and add them into the physics world
        computeContact(contactInfo);
    }
}

// Compute a contact (and add it to the physics world) for two colliding bodies
void CollisionDetection::computeContact(const ContactInfo* const contactInfo) {
    
    // Extract informations from the contact info structure
    const OBB* const obb1 = contactInfo->obb1;
    const OBB* const obb2 = contactInfo->obb2;
    Vector3D normal = contactInfo->normal;
    double penetrationDepth = contactInfo->penetrationDepth;

    const std::vector<Vector3D> obb1ExtremePoints = obb1->getExtremeVertices(normal);
    const std::vector<Vector3D> obb2ExtremePoints = obb2->getExtremeVertices(normal.getOpposite());
    unsigned int nbVerticesExtremeOBB1 = obb1ExtremePoints.size();
    unsigned int nbVerticesExtremeOBB2 = obb2ExtremePoints.size();
    assert(nbVerticesExtremeOBB1==1 || nbVerticesExtremeOBB1==2 || nbVerticesExtremeOBB1==4);
    assert(nbVerticesExtremeOBB2==1 || nbVerticesExtremeOBB2==2 || nbVerticesExtremeOBB2==4);
    assert(approxEqual(normal.length(), 1.0));

    // If it's a Vertex-Something contact
    if (nbVerticesExtremeOBB1 == 1) {
        // Create a new contact and add it to the physics world
        world->addConstraint(new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, obb1ExtremePoints.at(0)));
    }
    else if(nbVerticesExtremeOBB2 == 1) {  // If its a Vertex-Something contact
        // Create a new contact and add it to the physics world
        world->addConstraint(new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, obb2ExtremePoints.at(0)));
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

        // For each point of the set of contact points
        for (unsigned int i=0; i<contactSet.size(); i++) {
            // Create a new contact and add it to the physics world
            world->addConstraint(new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, contactSet.at(i)));
        }
    }
    else if(nbVerticesExtremeOBB1 == 2 && nbVerticesExtremeOBB2 == 4) {     // If it's an edge-face contact
        // Compute the projection of the edge of OBB1 onto the same plane of the face of OBB2
        std::vector<Vector3D> edge = projectPointsOntoPlane(obb1ExtremePoints, obb2ExtremePoints[0], normal);

        // Clip the edge of OBB1 using the face of OBB2
        std::vector<Vector3D> clippedEdge = clipSegmentWithRectangleInPlane(edge, obb2ExtremePoints);

        // TODO : Correct this bug
        // The following code is to correct a bug when the projected "edge" is not inside the clip rectangle
        // of obb1ExtremePoints. Therefore, we compute the nearest two points that are on the rectangle.
        if (clippedEdge.size() != 2) {
            edge.clear();
            edge.push_back(computeNearestPointOnRectangle(edge[0], obb2ExtremePoints));
            edge.push_back(computeNearestPointOnRectangle(edge[1], obb2ExtremePoints));
            clippedEdge = clipSegmentWithRectangleInPlane(edge, obb2ExtremePoints);
        }
        
        // Move the clipped edge halway between the edge of OBB1 and the face of OBB2
        clippedEdge = movePoints(clippedEdge, penetrationDepth/2.0 * normal.getOpposite());

        assert(clippedEdge.size() == 2);

        // For each point of the contact set
        for (unsigned int i=0; i<clippedEdge.size(); i++) {
            // Create a new contact and add it to the physics world
            world->addConstraint(new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, clippedEdge.at(i)));
        }
    }
    else if(nbVerticesExtremeOBB1 == 4 && nbVerticesExtremeOBB2 == 2) {     // If it's an edge-face contact
        // Compute the projection of the edge of OBB2 onto the same plane of the face of OBB1
        std::vector<Vector3D> edge = projectPointsOntoPlane(obb2ExtremePoints, obb1ExtremePoints[0], normal);

        // Clip the edge of OBB2 using the face of OBB1
        std::vector<Vector3D> clippedEdge = clipSegmentWithRectangleInPlane(edge, obb1ExtremePoints);

        // TODO : Correct this bug
        // The following code is to correct a bug when the projected "edge" is not inside the clip rectangle
        // of obb1ExtremePoints. Therefore, we compute the nearest two points that are on the rectangle.
        if (clippedEdge.size() != 2) {
            edge.clear();
            edge.push_back(computeNearestPointOnRectangle(edge[0], obb1ExtremePoints));
            edge.push_back(computeNearestPointOnRectangle(edge[1], obb1ExtremePoints));
            clippedEdge = clipSegmentWithRectangleInPlane(edge, obb1ExtremePoints);
        }
        
        // Move the clipped edge halfway between the face of OBB1 and the edge of OBB2
        clippedEdge = movePoints(clippedEdge, penetrationDepth/2.0 * normal);

        assert(clippedEdge.size() == 2);

        // For each point of the contact set
        for (unsigned int i=0; i<clippedEdge.size(); i++) {
            // Create a new contact and add it to the physics world
            world->addConstraint(new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, clippedEdge.at(i)));
        }
    }
    else {      // If it's a face-face contact
        // Compute the projection of the face vertices of OBB2 onto the plane of the face of OBB1
        std::vector<Vector3D> faceOBB2 = projectPointsOntoPlane(obb2ExtremePoints, obb1ExtremePoints[0], normal);

        // Clip the face of OBB2 using the face of OBB1
        std::vector<Vector3D> clippedFace = clipPolygonWithRectangleInPlane(faceOBB2, obb1ExtremePoints);

        // Move the clipped face halfway between the face of OBB1 and the face of OBB2
        clippedFace = movePoints(clippedFace, penetrationDepth/2.0 * normal);
        assert(clippedFace.size() >= 3);

        // For each point of the contact set
        for (unsigned int i=0; i<clippedFace.size(); i++) {
            // Create a new contact and add it to the physics world
            world->addConstraint(new Contact(obb1->getBodyPointer(), obb2->getBodyPointer(), normal, penetrationDepth, clippedFace.at(i)));
        }
    }
}
