/****************************************************************************
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
#include "LCPProjectedGaussSeidel.h"

using namespace reactphysics3d;

// Constructor
LCPProjectedGaussSeidel::LCPProjectedGaussSeidel(unsigned int maxIterations)
                        :maxIterations(maxIterations) {

}

// Destructor
LCPProjectedGaussSeidel::~LCPProjectedGaussSeidel() {

}

// Solve a LCP problem using the Projected-Gauss-Seidel algorithm
void LCPProjectedGaussSeidel::solve(const Matrix& A, const Vector& b, const Vector& lowLimits, const Vector& highLimits, Vector& x) {
    assert(A.getNbRow() == A.getNbColumn());
    assert(b.getNbComponent() == A.getNbColumn());
    assert(lowLimits.getNbComponent() == A.getNbColumn());
    assert(highLimits.getNbComponent() == A.getNbColumn());

    double delta;

    for (unsigned int k=1; k<=maxIterations; k++) {
        for (unsigned int i=0; i<A.getNbRow(); i++) {
            delta = 0.0;
            for (unsigned int j=0; j<i; j++) {
                delta += A.getValue(i,j) * x.getValue(j);
            }
            for (unsigned int j=i+1; j<A.getNbRow(); j++) {
                delta += A.getValue(i,j)*x.getValue(j);
            }
            x.setValue(i, (b.getValue(i) - delta)/A.getValue(i,i));

            // Clamping according to the limits
            if (x.getValue(i) > highLimits.getValue(i))  x.setValue(i, highLimits.getValue(i));
            if (x.getValue(i) < lowLimits.getValue(i))   x.setValue(i, lowLimits.getValue(i));
        }
    }
}
