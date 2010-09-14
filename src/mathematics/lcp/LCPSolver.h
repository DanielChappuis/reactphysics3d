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

#ifndef LCPSOLVER_H
#define LCPSOLVER_H

// Libraries
#include <vector>
#include <map>
#include "../Vector.h"
#include "../Matrix.h"
#include "../../body/Body.h"
#include "../../typeDefinitions.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class LCPSolver :
        This abstract class represents an algorithm to solve a Linear
        Complementary Problem (LCP). Given a matrix "A=J*B", a vector "b",
        a vector "lowLimit" of lower limits and a vector "highLimits" of
        upper limits. The goal is to find a vector "lambda" such that:

                w = Ax - b
                lowLimits <= lambda <= highLimits

        and one of the thre following conditions holds :

                lambda_i = lowLimits_i, w_i >= 0
                lambda_i = highLimits_i, w_i >= 0
                lowLimits_i < lambda_i < highLimits_i, w_i = 0

        Note that the matrix A is givent by the two matrices J and B with A=J*B.
        But only their sparse representations "J_sp" and "B_sp" are passed in
        arguments to solve() to be more efficient.
    -------------------------------------------------------------------
*/
class LCPSolver {
    protected:
        uint maxIterations;             // Maximum number of iterations
        Vector lambdaInit;              // Initial value for lambda at the beginning of the algorithm

    public:
        LCPSolver(uint maxIterations);                                                                              // Constructor
        virtual ~LCPSolver();                                                                                       // Destructor
        virtual void solve(Matrix1x6** J_sp, Vector6D** B_sp, uint nbConstraints,
                           uint nbBodies, Body*** const bodyMapping, std::map<Body*, uint> bodyNumberMapping,
                           const Vector& b, const Vector& lowLimits, const Vector& highLimits, Vector& lambda) const=0;                                       // Solve a LCP problem
        void setLambdaInit(const Vector& lambdaInit);                                                               // Set the initial lambda vector
        void setMaxIterations(uint maxIterations);                                                                  // Set the maximum number of iterations
};

// Set the initial lambda vector
inline void LCPSolver::setLambdaInit(const Vector& lambdaInit) {
    this->lambdaInit.changeSize(lambdaInit.getNbComponent());
    this->lambdaInit = lambdaInit;
}

// Set the maximum number of iterations
inline void LCPSolver::setMaxIterations(uint maxIterations) {
    assert(maxIterations > 0);
    this->maxIterations = maxIterations;
}

} // End of the ReactPhysics3D namespace

#endif
