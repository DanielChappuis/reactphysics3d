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

// Libraries
#include "Simplex.h"
#include <cfloat>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Simplex::Simplex() : bitsCurrentSimplex(0x0), allBits(0x0) {

}

// Destructor
Simplex::~Simplex() {

}

// Add a new support point of (A-B) into the simplex
// suppPointA : support point of object A in a direction -v
// suppPointB : support point of object B in a direction v
// point      : support point of object (A-B) => point = suppPointA - suppPointB
void Simplex::addPoint(const Vector3D& point, const Vector3D& suppPointA, const Vector3D& suppPointB) {
    assert(!isFull());

    lastFound = 0;
    lastFoundBit = 0x1;

    // Look for the bit corresponding to one of the four point that is not in
    // the current simplex
    while (overlap(bitsCurrentSimplex, lastFoundBit)) {
        lastFound++;
        lastFoundBit <<= 1;
    }

    assert(lastFound >= 0 && lastFound < 4);

    // Add the point into the simplex
    points[lastFound] = point;
    pointsLengthSquare[lastFound] = point.scalarProduct(point);
    allBits = bitsCurrentSimplex | lastFoundBit;

    // Update the cached values
    updateCache();
    
    // Compute the cached determinant values
    computeDeterminants();
    
    // Add the support points of objects A and B
    suppPointsA[lastFound] = suppPointA;
    suppPointsB[lastFound] = suppPointB;
}

// Return true if the point is in the simplex
bool Simplex::isPointInSimplex(const Vector3D& point) const {
    int i;
    Bits bit;

    // For each four possible points in the simplex
    for (i=0, bit = 0x1; i<4; i++, bit <<= 1) {
        // Check if the current point is in the simplex
        if (overlap(allBits, bit) && point == points[i]) return true;
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
        if (overlap(bitsCurrentSimplex, bit)) {
            
            // Compute the distance between two points in the possible simplex set
            diffLength[i][lastFound] = points[i] - points[lastFound];
            diffLength[lastFound][i] = diffLength[i][lastFound].getOpposite();

            // Compute the squared length of the vector distances from points in the possible simplex set
            normSquare[i][lastFound] = normSquare[lastFound][i] = diffLength[i][lastFound].scalarProduct(diffLength[i][lastFound]);
        }
    }
}

// Compute the cached determinant values
void Simplex::computeDeterminants() {
    det[lastFoundBit][lastFound] = 1.0;

    // If the current simplex is not empty
    if (!isEmpty()) {
        int i;
        Bits bitI;

        // For each possible four points in the simplex set
        for (i=0, bitI = 0x1; i<4; i++, bitI <<= 1) {

            // If the current point is in the simplex
            if (overlap(bitsCurrentSimplex, bitI)) {
                Bits bit2 = bitI | lastFoundBit;

                det[bit2][i] = diffLength[lastFound][i].scalarProduct(points[lastFound]);
                det[bit2][lastFound] = diffLength[i][lastFound].scalarProduct(points[i]);
      

                int j;
                Bits bitJ;

                for (j=0, bitJ = 0x1; j<i; j++, bitJ <<= 1) {
                    if (overlap(bitsCurrentSimplex, bitJ)) {
                        int k;
                        Bits bit3 = bitJ | bit2;

                        k = normSquare[i][j] < normSquare[lastFound][j] ? i : lastFound;
                        det[bit3][j] = det[bit2][i] * diffLength[k][j].scalarProduct(points[i]) +
                                       det[bit2][lastFound] * diffLength[k][j].scalarProduct(points[lastFound]);

                        k = normSquare[j][i] < normSquare[lastFound][i] ? j : lastFound;
                        det[bit3][i] = det[bitJ | lastFoundBit][j] * diffLength[k][i].scalarProduct(points[j]) +
                                       det[bitJ | lastFoundBit][lastFound] * diffLength[k][i].scalarProduct(points[lastFound]);

                        k = normSquare[i][lastFound] < normSquare[j][lastFound] ? i : j;
                        det[bit3][lastFound] = det[bitJ | bitI][j] * diffLength[k][lastFound].scalarProduct(points[j]) +
                                               det[bitJ | bitI][i] * diffLength[k][lastFound].scalarProduct(points[i]);
                    }
                }
            }
        }

        if (allBits == 0xf) {
            int k;

            k = normSquare[1][0] < normSquare[2][0] ? (normSquare[1][0] < normSquare[3][0] ? 1 : 3) : (normSquare[2][0] < normSquare[3][0] ? 2 : 3);
            det[0xf][0] = det[0xe][1] * diffLength[k][0].scalarProduct(points[1]) +
                        det[0xe][2] * diffLength[k][0].scalarProduct(points[2]) +
                        det[0xe][3] * diffLength[k][0].scalarProduct(points[3]);

            k = normSquare[0][1] < normSquare[2][1] ? (normSquare[0][1] < normSquare[3][1] ? 0 : 3) : (normSquare[2][1] < normSquare[3][1] ? 2 : 3);
            det[0xf][1] = det[0xd][0] * diffLength[k][1].scalarProduct(points[0]) +
                        det[0xd][2] * diffLength[k][1].scalarProduct(points[2]) +
                        det[0xd][3] * diffLength[k][1].scalarProduct(points[3]);

            k = normSquare[0][2] < normSquare[1][2] ? (normSquare[0][2] < normSquare[3][2] ? 0 : 3) : (normSquare[1][2] < normSquare[3][2] ? 1 : 3);
            det[0xf][2] = det[0xb][0] * diffLength[k][2].scalarProduct(points[0]) +
                        det[0xb][1] * diffLength[k][2].scalarProduct(points[1]) +
                        det[0xb][3] * diffLength[k][2].scalarProduct(points[3]);

            k = normSquare[0][3] < normSquare[1][3] ? (normSquare[0][3] < normSquare[2][3] ? 0 : 2) : (normSquare[1][3] < normSquare[2][3] ? 1 : 2);
            det[0xf][3] = det[0x7][0] * diffLength[k][3].scalarProduct(points[0]) +
                        det[0x7][1] * diffLength[k][3].scalarProduct(points[1]) +
                        det[0x7][2] * diffLength[k][3].scalarProduct(points[2]);
        }
    }
}

// Return true if the subset is a proper subset
// A proper subset X is a subset where for all point "y_i" in X, we have
// detX_i value bigger than zero
bool Simplex::isProperSubset(Bits subset) const {
    int i;
    Bits bit;

    // For each four point of the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<=1) {
        if (overlap(subset, bit) && det[subset][i] <= 0.0) {
            return false;
        }
    }

    return true;
}

// Return true if the set is affinely dependent meaning that a point of the set
// is an affine combination of other points in the set
bool Simplex::isAffinelyDependent() const {
    double sum = 0.0;
    int i;
    Bits bit;

    // For each four point of the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<= 1) {
        if (overlap(allBits, bit)) {
            sum += det[allBits][i];
        }
    }

    return (sum <= 0.0);
}

// Return true if the subset is a valid one for the closest point computation
// A subset X is valid if :
//    1. delta(X)_i > 0 for each "i" in I_x and
//    2. delta(X U {y_j})_j <= 0 for each "j" not in I_x_
bool Simplex::isValidSubset(Bits subset) const {
    int i;
    Bits bit;

    // For each four point in the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<= 1) {
        if (overlap(allBits, bit)) {
            // If the current point is in the subset
            if (overlap(subset, bit)) {
                // If one delta(X)_i is smaller or equal to zero
                if (det[subset][i] <= 0.0) {
                    // The subset is not valid
                    return false;
                }
            }
            // If the point is not in the subset and the value delta(X U {y_j})_j
            // is bigger than zero
            else if (det[subset | bit][i] > 0.0) {
                // The subset is not valid
                return false;
            }
        }
    }

    return true;
}

// Compute the closest points "pA" and "pB" of object A and B
// The points are computed as follows :
//      pA = sum(lambda_i * a_i)    where "a_i" are the support points of object A
//      pB = sum(lambda_i * b_i)    where "b_i" are the support points of object B
//      with lambda_i = deltaX_i / deltaX
void Simplex::computeClosestPointsOfAandB(Vector3D& pA, Vector3D& pB) const {
    double deltaX = 0.0;
    pA.setAllValues(0.0, 0.0, 0.0);
    pB.setAllValues(0.0, 0.0, 0.0);
    int i;
    Bits bit;

    // For each four points in the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<= 1) {
        // If the current point is part of the simplex
        if (overlap(bitsCurrentSimplex, bit)) {
            deltaX += det[bitsCurrentSimplex][i];
            pA = pA + det[bitsCurrentSimplex][i] * suppPointsA[i];
            pB = pB + det[bitsCurrentSimplex][i] * suppPointsB[i];
        }
    }

    assert(deltaX > 0.0);
    double factor = 1.0 / deltaX;
    pA = factor * pA;
    pB = factor * pB;
}

// Compute the closest point "v" to the origin of the current simplex
// This method executes the Jonhnson's algorithm for computing the point
// "v" of simplex that is closest to the origin. The method returns true
// if a closest point has been found.
bool Simplex::computeClosestPoint(Vector3D& v) {
    Bits subset;

    // For each possible simplex set
    for (subset=bitsCurrentSimplex; subset != 0x0; subset--) {
        // If the simplex is a subset of the current simplex and is valid for the Johnson's
        // algorithm test
        if (isSubset(subset, bitsCurrentSimplex) && isValidSubset(subset | lastFoundBit)) {
           bitsCurrentSimplex = subset | lastFoundBit;              // Add the last added point to the current simplex
           v = computeClosestPointForSubset(bitsCurrentSimplex);    // Compute the closest point in the simplex
           return true;
        }
    }

    // If the simplex that contains only the last added point is valid for the
    // Johnson's algorithm test
    if (isValidSubset(lastFoundBit)) {
        bitsCurrentSimplex = lastFoundBit;      // Set the current simplex to the set that contains only the last added point
        v = points[lastFound];                  // The closest point of the simplex "v" is the last added point
        return true;
    }

    // The algorithm failed to found a point
    return false;
}

// Backup the closest point
void Simplex::backupClosestPointInSimplex(Vector3D& v) {
    double minDistSquare = DBL_MAX;
    Bits bit;

    for (bit = allBits; bit != 0x0; bit--) {
        if (isSubset(bit, allBits) && isProperSubset(bit)) {
            Vector3D u = computeClosestPointForSubset(bit);
            double distSquare = u.scalarProduct(u);
            if (distSquare < minDistSquare) {
                minDistSquare = distSquare;
                bitsCurrentSimplex = bit;
                v = u;
            }
        }
    }
}

// Return the closest point "v" in the convex hull of the points in the subset
// represented by the bits "subset"
Vector3D Simplex::computeClosestPointForSubset(Bits subset) const {
    Vector3D v(0.0, 0.0, 0.0);      // Closet point v = sum(lambda_i * points[i])
    double maxLenSquare = 0.0;
    double deltaX = 0.0;            // deltaX = sum of all det[subset][i]
    int i;
    Bits bit;

    // For each four point in the possible simplex set
    for (i=0, bit=0x1; i<4; i++, bit <<= 1) {
        // If the current point is in the subset
        if (overlap(subset, bit)) {
            // deltaX = sum of all det[subset][i]
            deltaX += det[subset][i];

            if (maxLenSquare < pointsLengthSquare[i]) {
                maxLenSquare = pointsLengthSquare[i];
            }

            // Closest point v = sum(lambda_i * points[i])
            v = v + det[subset][i] * points[i];
        }
    }

    assert(deltaX > 0.0);

    // Return the closet point "v" in the convex hull for the given subset
    return (1.0 / deltaX) * v;
}