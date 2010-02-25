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

#ifndef LCPPROJECTEDGAUSSSEIDEL_H
#define LCPPROJECTEDGAUSSSEIDEL_H

// Libraries
#include "LCPSolver.h"

namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class LCPProjectedGaussSeidel :
        This class implements the Projected-Gauss-Seidel (PGS)
        algorithm in order to solve a LCP problem. This class inherits
        from the LCPSolver class.
    -------------------------------------------------------------------
*/
class LCPProjectedGaussSeidel : public LCPSolver {
    protected:
        unsigned int maxIterations;        // Maximum number of iterations

    public:
        LCPProjectedGaussSeidel(unsigned int maxIterations);                                                                    // Constructor
        virtual ~LCPProjectedGaussSeidel();                                                                                     // Destructor
        virtual void solve(const Matrix& A, const Vector& b, const Vector& lowLimits, const Vector& highLimits, Vector& x);     // Solve a LCP problem using Projected-Gauss-Seidel algorithm

};

} // End of the ReactPhysics3D namespace

#endif
