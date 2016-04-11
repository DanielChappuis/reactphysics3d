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

// Libraries
#include "Simplex.h"
#include <cfloat>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Simplex::Simplex() : mBitsCurrentSimplex(0x0), mAllBits(0x0) {

}

// Destructor
Simplex::~Simplex() {

}

// Add a new support point of (A-B) into the simplex
/// suppPointA : support point of object A in a direction -v
/// suppPointB : support point of object B in a direction v
/// point      : support point of object (A-B) => point = suppPointA - suppPointB
void Simplex::addPoint(const Vector3& point, const Vector3& suppPointA, const Vector3& suppPointB) {
    assert(!isFull());

    mLastFound = 0;
    mLastFoundBit = 0x1;

    // Look for the bit corresponding to one of the four point that is not in
    // the current simplex
    while (overlap(mBitsCurrentSimplex, mLastFoundBit)) {
        mLastFound++;
        mLastFoundBit <<= 1;
    }

    assert(mLastFound < 4);

    // Add the point into the simplex
    mPoints[mLastFound] = point;
    mPointsLengthSquare[mLastFound] = point.dot(point);
    mAllBits = mBitsCurrentSimplex | mLastFoundBit;

    // Update the cached values
    updateCache();
    
    // Compute the cached determinant values
    computeDeterminants();
    
    // Add the support points of objects A and B
    mSuppPointsA[mLastFound] = suppPointA;
    mSuppPointsB[mLastFound] = suppPointB;
}

// Return true if the point is in the simplex
bool Simplex::isPointInSimplex(const Vector3& point) const {
    int i;
    Bits bit;

    // For each four possible points in the simplex
    for (i=0, bit = 0x1; i<4; i++, bit <<= 1) {
        // Check if the current point is in the simplex
        if (overlap(mAllBits, bit) && point == mPoints[i]) return true;
    }

    return false;
}

// Update the cached values used during the GJK algorithm
void Simplex::updateCache() {
    int i;
    Bits bit;

    // For each of the four possible points of the simplex
    for (i=0, bit = 0x1; i<4; i++, bit <<= 1) {
        // If the current points is in the simplex
        if (overlap(mBitsCurrentSimplex, bit)) {
            
            // Compute the distance between two points in the possible simplex set
            mDiffLength[i][mLastFound] = mPoints[i] - mPoints[mLastFound];
            mDiffLength[mLastFound][i] = -mDiffLength[i][mLastFound];

            // Compute the squared length of the vector
            // distances from points in the possible simplex set
            mNormSquare[i][mLastFound] = mNormSquare[mLastFound][i] =
                                         mDiffLength[i][mLastFound].dot(mDiffLength[i][mLastFound]);
        }
    }
}

// Return the points of the simplex
unsigned int Simplex::getSimplex(Vector3* suppPointsA, Vector3* suppPointsB,
                                 Vector3* points) const {
    unsigned int nbVertices = 0;
    int i;
    Bits bit;

    // For each four point in the possible simplex
    for (i=0, bit=0x1; i<4; i++, bit <<=1) {

        // If the current point is in the simplex
        if (overlap(mBitsCurrentSimplex, bit)) {

            // Store the points
            suppPointsA[nbVertices] = this->mSuppPointsA[nbVertices];
            suppPointsB[nbVertices] = this->mSuppPointsB[nbVertices];
            points[nbVertices] = this->mPoints[nbVertices];

            nbVertices++;
        }
    }

    // Return the number of points in the simplex
    return nbVertices;
}

// Compute the cached determinant values
void Simplex::computeDeterminants() {
    mDet[mLastFoundBit][mLastFound] = 1.0;

    // If the current simplex is not empty
    if (!isEmpty()) {
        int i;
        Bits bitI;

        // For each possible four points in the simplex set
        for (i=0, bitI = 0x1; i<4; i++, bitI <<= 1) {

            // If the current point is in the simplex
            if (overlap(mBitsCurrentSimplex, bitI)) {
                Bits bit2 = bitI | mLastFoundBit;

                mDet[bit2][i] = mDiffLength[mLastFound][i].dot(mPoints[mLastFound]);
                mDet[bit2][mLastFound] = mDiffLength[i][mLastFound].dot(mPoints[i]);
      

                int j;
                Bits bitJ;

                for (j=0, bitJ = 0x1; j<i; j++, bitJ <<= 1) {
                    if (overlap(mBitsCurrentSimplex, bitJ)) {
                        int k;
                        Bits bit3 = bitJ | bit2;

                        k = mNormSquare[i][j] < mNormSquare[mLastFound][j] ? i : mLastFound;
                        mDet[bit3][j] = mDet[bit2][i] * mDiffLength[k][j].dot(mPoints[i]) +
                                       mDet[bit2][mLastFound] *
                                       mDiffLength[k][j].dot(mPoints[mLastFound]);

                        k = mNormSquare[j][i] < mNormSquare[mLastFound][i] ? j : mLastFound;
                        mDet[bit3][i] = mDet[bitJ | mLastFoundBit][j] *
                                        mDiffLength[k][i].dot(mPoints[j]) +
                                        mDet[bitJ | mLastFoundBit][mLastFound] *
                                        mDiffLength[k][i].dot(mPoints[mLastFound]);

                        k = mNormSquare[i][mLastFound] < mNormSquare[j][mLastFound] ? i : j;
                        mDet[bit3][mLastFound] = mDet[bitJ | bitI][j] *
                                                 mDiffLength[k][mLastFound].dot(mPoints[j]) +
                                                 mDet[bitJ | bitI][i] *
                                                 mDiffLength[k][mLastFound].dot(mPoints[i]);
                    }
                }
            }
        }

        if (mAllBits == 0xf) {
            int k;

            k = mNormSquare[1][0] < mNormSquare[2][0] ?
                                   (mNormSquare[1][0] < mNormSquare[3][0] ? 1 : 3) :
                                   (mNormSquare[2][0] < mNormSquare[3][0] ? 2 : 3);
            mDet[0xf][0] = mDet[0xe][1] * mDiffLength[k][0].dot(mPoints[1]) +
                        mDet[0xe][2] * mDiffLength[k][0].dot(mPoints[2]) +
                        mDet[0xe][3] * mDiffLength[k][0].dot(mPoints[3]);

            k = mNormSquare[0][1] < mNormSquare[2][1] ?
                                   (mNormSquare[0][1] < mNormSquare[3][1] ? 0 : 3) :
                                   (mNormSquare[2][1] < mNormSquare[3][1] ? 2 : 3);
            mDet[0xf][1] = mDet[0xd][0] * mDiffLength[k][1].dot(mPoints[0]) +
                        mDet[0xd][2] * mDiffLength[k][1].dot(mPoints[2]) +
                        mDet[0xd][3] * mDiffLength[k][1].dot(mPoints[3]);

            k = mNormSquare[0][2] < mNormSquare[1][2] ?
                                   (mNormSquare[0][2] < mNormSquare[3][2] ? 0 : 3) :
                                   (mNormSquare[1][2] < mNormSquare[3][2] ? 1 : 3);
            mDet[0xf][2] = mDet[0xb][0] * mDiffLength[k][2].dot(mPoints[0]) +
                        mDet[0xb][1] * mDiffLength[k][2].dot(mPoints[1]) +
                        mDet[0xb][3] * mDiffLength[k][2].dot(mPoints[3]);

            k = mNormSquare[0][3] < mNormSquare[1][3] ?
                                   (mNormSquare[0][3] < mNormSquare[2][3] ? 0 : 2) :
                                   (mNormSquare[1][3] < mNormSquare[2][3] ? 1 : 2);
            mDet[0xf][3] = mDet[0x7][0] * mDiffLength[k][3].dot(mPoints[0]) +
                        mDet[0x7][1] * mDiffLength[k][3].dot(mPoints[1]) +
                        mDet[0x7][2] * mDiffLength[k][3].dot(mPoints[2]);
        }
    }
}

// Return true if the subset is a proper subset.
/// A proper subset X is a subset where for all point "y_i" in X, we have
/// detX_i value bigger than zero
bool Simplex::isProperSubset(Bits subset) const {
    int i;
    Bits bit;

    // For each four point of the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<=1) {
        if (overlap(subset, bit) && mDet[subset][i] <= 0.0) {
            return false;
        }
    }

    return true;
}

// Return true if the set is affinely dependent.
/// A set if affinely dependent if a point of the set
/// is an affine combination of other points in the set
bool Simplex::isAffinelyDependent() const {
    decimal sum = 0.0;
    int i;
    Bits bit;

    // For each four point of the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<= 1) {
        if (overlap(mAllBits, bit)) {
            sum += mDet[mAllBits][i];
        }
    }

    return (sum <= 0.0);
}

// Return true if the subset is a valid one for the closest point computation.
/// A subset X is valid if :
///    1. delta(X)_i > 0 for each "i" in I_x and
///    2. delta(X U {y_j})_j <= 0 for each "j" not in I_x_
bool Simplex::isValidSubset(Bits subset) const {
    int i;
    Bits bit;

    // For each four point in the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<= 1) {
        if (overlap(mAllBits, bit)) {
            // If the current point is in the subset
            if (overlap(subset, bit)) {
                // If one delta(X)_i is smaller or equal to zero
                if (mDet[subset][i] <= 0.0) {
                    // The subset is not valid
                    return false;
                }
            }
            // If the point is not in the subset and the value delta(X U {y_j})_j
            // is bigger than zero
            else if (mDet[subset | bit][i] > 0.0) {
                // The subset is not valid
                return false;
            }
        }
    }

    return true;
}

// Compute the closest points "pA" and "pB" of object A and B.
/// The points are computed as follows :
///      pA = sum(lambda_i * a_i)    where "a_i" are the support points of object A
///      pB = sum(lambda_i * b_i)    where "b_i" are the support points of object B
///      with lambda_i = deltaX_i / deltaX
void Simplex::computeClosestPointsOfAandB(Vector3& pA, Vector3& pB) const {
    decimal deltaX = 0.0;
    pA.setAllValues(0.0, 0.0, 0.0);
    pB.setAllValues(0.0, 0.0, 0.0);
    int i;
    Bits bit;

    // For each four points in the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<= 1) {
        // If the current point is part of the simplex
        if (overlap(mBitsCurrentSimplex, bit)) {
            deltaX += mDet[mBitsCurrentSimplex][i];
            pA += mDet[mBitsCurrentSimplex][i] * mSuppPointsA[i];
            pB += mDet[mBitsCurrentSimplex][i] * mSuppPointsB[i];
        }
    }

    assert(deltaX > 0.0);
    decimal factor = decimal(1.0) / deltaX;
    pA *= factor;
    pB *= factor;
}

// Compute the closest point "v" to the origin of the current simplex.
/// This method executes the Jonhnson's algorithm for computing the point
/// "v" of simplex that is closest to the origin. The method returns true
/// if a closest point has been found.
bool Simplex::computeClosestPoint(Vector3& v) {
    Bits subset;

    // For each possible simplex set
    for (subset=mBitsCurrentSimplex; subset != 0x0; subset--) {
        // If the simplex is a subset of the current simplex and is valid for the Johnson's
        // algorithm test
        if (isSubset(subset, mBitsCurrentSimplex) && isValidSubset(subset | mLastFoundBit)) {
           mBitsCurrentSimplex = subset | mLastFoundBit;              // Add the last added point to the current simplex
           v = computeClosestPointForSubset(mBitsCurrentSimplex);    // Compute the closest point in the simplex
           return true;
        }
    }

    // If the simplex that contains only the last added point is valid for the Johnson's algorithm test
    if (isValidSubset(mLastFoundBit)) {
        mBitsCurrentSimplex = mLastFoundBit;                  // Set the current simplex to the set that contains only the last added point
        mMaxLengthSquare = mPointsLengthSquare[mLastFound];    // Update the maximum square length
        v = mPoints[mLastFound];                              // The closest point of the simplex "v" is the last added point
        return true;
    }

    // The algorithm failed to found a point
    return false;
}

// Backup the closest point
void Simplex::backupClosestPointInSimplex(Vector3& v) {
    decimal minDistSquare = DECIMAL_LARGEST;
    Bits bit;

    for (bit = mAllBits; bit != 0x0; bit--) {
        if (isSubset(bit, mAllBits) && isProperSubset(bit)) {
            Vector3 u = computeClosestPointForSubset(bit);
            decimal distSquare = u.dot(u);
            if (distSquare < minDistSquare) {
                minDistSquare = distSquare;
                mBitsCurrentSimplex = bit;
                v = u;
            }
        }
    }
}

// Return the closest point "v" in the convex hull of the points in the subset
// represented by the bits "subset"
Vector3 Simplex::computeClosestPointForSubset(Bits subset) {
    Vector3 v(0.0, 0.0, 0.0);      // Closet point v = sum(lambda_i * points[i])
    mMaxLengthSquare = 0.0;
    decimal deltaX = 0.0;            // deltaX = sum of all det[subset][i]
    int i;
    Bits bit;

    // For each four point in the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<= 1) {
        // If the current point is in the subset
        if (overlap(subset, bit)) {
            // deltaX = sum of all det[subset][i]
            deltaX += mDet[subset][i];

            if (mMaxLengthSquare < mPointsLengthSquare[i]) {
                mMaxLengthSquare = mPointsLengthSquare[i];
            }

            // Closest point v = sum(lambda_i * points[i])
            v += mDet[subset][i] * mPoints[i];
        }
    }

    assert(deltaX > 0.0);

    // Return the closet point "v" in the convex hull for the given subset
    return (decimal(1.0) / deltaX) * v;
}
