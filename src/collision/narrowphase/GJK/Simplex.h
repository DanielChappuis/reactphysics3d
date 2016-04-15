/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_SIMPLEX_H
#define	REACTPHYSICS3D_SIMPLEX_H

// Libraries
#include "mathematics/mathematics.h"
#include <vector>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Type definitions
typedef unsigned int Bits;

// Class Simplex
/**
 * This class represents a simplex which is a set of 3D points. This
 * class is used in the GJK algorithm. This implementation is based on
 * the implementation discussed in the book "Collision Detection in 3D
 * Environments". This class implements the Johnson's algorithm for
 * computing the point of a simplex that is closest to the origin and also
 * the smallest simplex needed to represent that closest point.
 */
class Simplex {

    private:

        // -------------------- Attributes -------------------- //

        /// Current points
        Vector3 mPoints[4];

        /// pointsLengthSquare[i] = (points[i].length)^2
        decimal mPointsLengthSquare[4];

        /// Maximum length of pointsLengthSquare[i]
        decimal mMaxLengthSquare;

        /// Support points of object A in local coordinates
        Vector3 mSuppPointsA[4];

        /// Support points of object B in local coordinates
        Vector3 mSuppPointsB[4];

        /// diff[i][j] contains points[i] - points[j]
        Vector3 mDiffLength[4][4];

        /// Cached determinant values
        decimal mDet[16][4];

        /// norm[i][j] = (diff[i][j].length())^2
        decimal mNormSquare[4][4];

        /// 4 bits that identify the current points of the simplex
        /// For instance, 0101 means that points[1] and points[3] are in the simplex
        Bits mBitsCurrentSimplex;

        /// Number between 1 and 4 that identify the last found support point
        Bits mLastFound;

        /// Position of the last found support point (lastFoundBit = 0x1 << lastFound)
        Bits mLastFoundBit;

        /// allBits = bitsCurrentSimplex | lastFoundBit;
        Bits mAllBits;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        Simplex(const Simplex& simplex);

        /// Private assignment operator
        Simplex& operator=(const Simplex& simplex);

        /// Return true if some bits of "a" overlap with bits of "b"
        bool overlap(Bits a, Bits b) const;

        /// Return true if the bits of "b" is a subset of the bits of "a"
        bool isSubset(Bits a, Bits b) const;

        /// Return true if the subset is a valid one for the closest point computation.
        bool isValidSubset(Bits subset) const;

        /// Return true if the subset is a proper subset.
        bool isProperSubset(Bits subset) const;

        /// Update the cached values used during the GJK algorithm
        void updateCache();

        /// Compute the cached determinant values
        void computeDeterminants();

        /// Return the closest point "v" in the convex hull of a subset of points
        Vector3 computeClosestPointForSubset(Bits subset);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Simplex();

        /// Destructor
        ~Simplex();

        /// Return true if the simplex contains 4 points
        bool isFull() const;

        /// Return true if the simplex is empty
        bool isEmpty() const;

        /// Return the points of the simplex
        unsigned int getSimplex(Vector3* mSuppPointsA, Vector3* mSuppPointsB,
                                Vector3* mPoints) const;

        /// Return the maximum squared length of a point
        decimal getMaxLengthSquareOfAPoint() const;

        /// Add a new support point of (A-B) into the simplex.
        void addPoint(const Vector3& point, const Vector3& suppPointA, const Vector3& suppPointB);

        /// Return true if the point is in the simplex
        bool isPointInSimplex(const Vector3& point) const;

        /// Return true if the set is affinely dependent
        bool isAffinelyDependent() const;

        /// Backup the closest point
        void backupClosestPointInSimplex(Vector3& point);

        /// Compute the closest points "pA" and "pB" of object A and B.
        void computeClosestPointsOfAandB(Vector3& pA, Vector3& pB) const;

        /// Compute the closest point to the origin of the current simplex.
        bool computeClosestPoint(Vector3& v);
};

// Return true if some bits of "a" overlap with bits of "b"
inline bool Simplex::overlap(Bits a, Bits b) const {
    return ((a & b) != 0x0);
}

// Return true if the bits of "b" is a subset of the bits of "a"
inline bool Simplex::isSubset(Bits a, Bits b) const {
    return ((a & b) == a);
}

// Return true if the simplex contains 4 points
inline bool Simplex::isFull() const {
    return (mBitsCurrentSimplex == 0xf);
}

// Return true if the simple is empty
inline bool Simplex::isEmpty() const {
    return (mBitsCurrentSimplex == 0x0);
}

// Return the maximum squared length of a point
inline decimal Simplex::getMaxLengthSquareOfAPoint() const {
    return mMaxLengthSquare;
}

}

#endif
