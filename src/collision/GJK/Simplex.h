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

#ifndef SIMPLEX_H
#define	SIMPLEX_H

// Libraries
#include "../../mathematics/mathematics.h"
#include <vector>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Type definitions
typedef unsigned int Bits;

/*  -------------------------------------------------------------------
    Class Simplex :
        This class represents a simplex which is a set of 3D points. This
        class is used in the GJK algorithm. This implementation is based on
        the implementation discussed in the book "Collision Detection in 3D
        Environments". This class implements the Johnson's algorithm for
        computing the point of a simplex that is closest to the origin and also
        the smallest simplex needed to represent that closest point.
    -------------------------------------------------------------------
*/
class Simplex {
    private:
        Vector3D points[4];                 // Current points
        double pointsLengthSquare[4];       // pointsLengthSquare[i] = (points[i].length)^2
        double maxLengthSquare;             // Maximum length of pointsLengthSquare[i]
        Vector3D suppPointsA[4];            // Support points of object A in local coordinates
        Vector3D suppPointsB[4];            // Support points of object B in local coordinates
        Vector3D diffLength[4][4];          // diff[i][j] contains points[i] - points[j]
        double det[16][4];                  // Cached determinant values
        double normSquare[4][4];            // norm[i][j] = (diff[i][j].length())^2
        Bits bitsCurrentSimplex;            // 4 bits that identify the current points of the simplex
                                            // For instance, 0101 means that points[1] and points[3] are in the simplex
        Bits lastFound;                     // Number between 1 and 4 that identify the last found support point
        Bits lastFoundBit;                  // Position of the last found support point (lastFoundBit = 0x1 << lastFound)
        Bits allBits;                       // allBits = bitsCurrentSimplex | lastFoundBit;

        bool overlap(Bits a, Bits b) const;                         // Return true if some bits of "a" overlap with bits of "b"
        bool isSubset(Bits a, Bits b) const;                        // Return true if the bits of "b" is a subset of the bits of "a"
        bool isValidSubset(Bits subset) const;                      // Return true if the subset is a valid one for the closest point computation
        bool isProperSubset(Bits subset) const;                     // Return true if the subset is a proper subset
        void updateCache();                                         // Update the cached values used during the GJK algorithm
        void computeDeterminants();                                 // Compute the cached determinant values
        Vector3D computeClosestPointForSubset(Bits subset);         // Return the closest point "v" in the convex hull of a subset of points

    public:
        Simplex();                          // Constructor
        ~Simplex();                         // Destructor

        bool isFull() const;                                                                            // Return true if the simplex contains 4 points
        bool isEmpty() const;                                                                           // Return true if the simple is empty
        unsigned int getSimplex(Vector3D* suppPointsA, Vector3D* suppPointsB, Vector3D* points) const;  // Return the points of the simplex
        double getMaxLengthSquareOfAPoint() const;                                                      // Return the maximum squared length of a point
        void addPoint(const Vector3D& point, const Vector3D& suppPointA, const Vector3D& suppPointB);   // Addd a point to the simplex
        bool isPointInSimplex(const Vector3D& point) const;                                             // Return true if the point is in the simplex
        bool isAffinelyDependent() const;                                                               // Return true if the set is affinely dependent
        void backupClosestPointInSimplex(Vector3D& point);                                              // Backup the closest point
        void computeClosestPointsOfAandB(Vector3D& pA, Vector3D& pB) const;                             // Compute the closest points of object A and B
        bool computeClosestPoint(Vector3D& v);                                                          // Compute the closest point to the origin of the current simplex
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
    return (bitsCurrentSimplex == 0xf);
}

// Return true if the simple is empty
inline bool Simplex::isEmpty() const {
    return (bitsCurrentSimplex == 0x0);
}

// Return the maximum squared length of a point
inline double Simplex::getMaxLengthSquareOfAPoint() const {
    return maxLengthSquare;
}

} // End of the ReactPhysics3D namespace

#endif