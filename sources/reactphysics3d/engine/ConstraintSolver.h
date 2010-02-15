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

#ifndef CONSTRAINTSOLVER_H
#define CONSTRAINTSOLVER_H

// Libraries
#include "../constraint/Constraint.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

 /*  -------------------------------------------------------------------
    Class ConstrainSolver :
        This class represents the constraint solver. The goal is to
        solve A constraint-base LCP problem.
    -------------------------------------------------------------------
*/
class ConstraintSolver {
    protected:
        void allocate(std::vector<Constraint*>& constraints, double dt);    // Allocate all the matrices needed to solve the LCP problem
        void fillInMatrices(std::vector<Constraint*> constraints);         // Fill in all the matrices needed to solve the LCP problem

    public:
        ConstraintSolver();                                                 // Constructor
        virtual ~ConstraintSolver();                                        // Destructor
        void solve(std::vector<Constraint*>& constraints, double dt);       // Solve the current LCP problem
};

} // End of ReactPhysics3D namespace

#endif
