/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef CONTACT_H
#define CONTACT_H

// Libraries
#include "Constraint.h"
#include "../collision/ContactInfo.h"
#include "../body/RigidBody.h"
#include "../configuration.h"
#include "../mathematics/mathematics.h"
#include "../memory/MemoryPool.h"
#include "../configuration.h"

#if defined(VISUAL_DEBUG)
	#if defined(APPLE_OS)
		#include <GLUT/glut.h>
		#include <OpenGL/gl.h>
	#elif defined(WINDOWS_OS)
		#include <GL/glut.h>
		#include <GL/gl.h>
	#elif defined(LINUX_OS)
        #include <GL/freeglut.h>
        #include <GL/gl.h>
    #endif
#endif

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Contact :
        This class represents a collision contact between two bodies in
        the physics engine. The contact class inherits from the
        Constraint class. Each Contact represent a contact between two bodies
        and contains the two contact points on each body. The contact has 3
        mathematical constraints (1 for the contact constraint, and 2
        for the friction constraints).
    -------------------------------------------------------------------
*/
class Contact : public Constraint {

    protected :

        // -------------------- Attributes -------------------- //

        // Normal vector of the contact (From body1 toward body2) in world space
        const Vector3 mNormal;

        // Penetration depth
        decimal mPenetrationDepth;

        // Contact point on body 1 in local space of body 1
        const Vector3 mLocalPointOnBody1;

        // Contact point on body 2 in local space of body 2
        const Vector3 mLocalPointOnBody2;

        // Contact point on body 1 in world space
        Vector3 mWorldPointOnBody1;

        // Contact point on body 2 in world space
        Vector3 mWorldPointOnBody2;

        // Two orthogonal vectors that span the tangential friction plane
        std::vector<Vector3> mFrictionVectors;

        decimal mMu_mc_g;
        
        // -------------------- Methods -------------------- //

        // Private copy-constructor
        Contact(const Contact& contact);

        // Private assignment operator
        Contact& operator=(const Contact& contact);

        // Compute the two friction vectors that span the tangential friction plane
        void computeFrictionVectors();

    public :

        // -------------------- Methods -------------------- //

        // Constructor
        Contact(RigidBody* const body1, RigidBody* const body2, const ContactInfo* contactInfo);

        // Destructor
        virtual ~Contact();

        // Return the normal vector of the contact
        Vector3 getNormal() const;

        // Set the penetration depth of the contact
        void setPenetrationDepth(decimal penetrationDepth);

        // Return the contact local point on body 1
        Vector3 getLocalPointOnBody1() const;

        // Return the contact local point on body 2
        Vector3 getLocalPointOnBody2() const;

        // Return the contact world point on body 1
        Vector3 getWorldPointOnBody1() const;

        // Return the contact world point on body 2
        Vector3 getWorldPointOnBody2() const;

        // Set the contact world point on body 1
        void setWorldPointOnBody1(const Vector3& worldPoint);

        // Set the contact world point on body 2
        void setWorldPointOnBody2(const Vector3& worldPoint);

        // Compute the jacobian matrix for all mathematical constraints
        virtual void computeJacobian(int noConstraint,
                                     decimal J_SP[NB_MAX_CONSTRAINTS][2*6]) const;

        // Compute the lowerbounds values for all the mathematical constraints
        virtual void computeLowerBound(int noConstraint,
                                       decimal lowerBounds[NB_MAX_CONSTRAINTS]) const;

        // Compute the upperbounds values for all the mathematical constraints
        virtual void computeUpperBound(int noConstraint,
                                       decimal upperBounds[NB_MAX_CONSTRAINTS]) const;

        // Compute the error values for all the mathematical constraints
        virtual void computeErrorValue(int noConstraint, decimal errorValues[]) const;

        void computeErrorPenetration(decimal& error);

        void computeJacobianPenetration(decimal J_spBody1[6], decimal J_spBody2[6]);

        void computeJacobianFriction1(decimal J_spBody1[6], decimal J_spBody2[6]);

        void computeJacobianFriction2(decimal J_spBody1[6], decimal J_spBody2[6]);

        void computeLowerBoundPenetration(decimal& lowerBound);
        void computeLowerBoundFriction1(decimal& lowerBound);
        void computeLowerBoundFriction2(decimal& lowerBound);

        void computeUpperBoundPenetration(decimal& upperBound);
        void computeUpperBoundFriction1(decimal& upperBound);
        void computeUpperBoundFriction2(decimal& upperBound);

        // Return the penetration depth
        decimal getPenetrationDepth() const;

        #ifdef VISUAL_DEBUG
            // Draw the contact (for debugging)
           void draw() const;
        #endif
};

// Compute the two unit orthogonal vectors "v1" and "v2" that span the tangential friction plane
// The two vectors have to be such that : v1 x v2 = contactNormal
inline void Contact::computeFrictionVectors() {
    // Delete the current friction vectors
    mFrictionVectors.clear();

    // Compute the first orthogonal vector
    Vector3 vector1 = mNormal.getOneOrthogonalVector();
    mFrictionVectors.push_back(vector1);

    // Compute the second orthogonal vector using the cross product
    mFrictionVectors.push_back(mNormal.cross(vector1));
}

// Return the normal vector of the contact
inline Vector3 Contact::getNormal() const {
    return mNormal;
}

// Set the penetration depth of the contact
inline void Contact::setPenetrationDepth(decimal penetrationDepth) {
    this->mPenetrationDepth = penetrationDepth;
}

// Return the contact point on body 1
inline Vector3 Contact::getLocalPointOnBody1() const {
    return mLocalPointOnBody1;
}

// Return the contact point on body 2
inline Vector3 Contact::getLocalPointOnBody2() const {
    return mLocalPointOnBody2;
}

// Return the contact world point on body 1
inline Vector3 Contact::getWorldPointOnBody1() const {
    return mWorldPointOnBody1;
}

// Return the contact world point on body 2
inline Vector3 Contact::getWorldPointOnBody2() const {
    return mWorldPointOnBody2;
}

// Set the contact world point on body 1
inline void Contact::setWorldPointOnBody1(const Vector3& worldPoint) {
    mWorldPointOnBody1 = worldPoint;
}

// Set the contact world point on body 2
inline void Contact::setWorldPointOnBody2(const Vector3& worldPoint) {
    mWorldPointOnBody2 = worldPoint;
}

// Return the penetration depth of the contact
inline decimal Contact::getPenetrationDepth() const {
    return mPenetrationDepth;
}


#ifdef VISUAL_DEBUG
inline void Contact::draw() const {
    glColor3f(1.0, 0.0, 0.0);
    glutSolidSphere(0.3, 20, 20);
}
#endif 

} // End of the ReactPhysics3D namespace

#endif
