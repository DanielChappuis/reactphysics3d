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

#ifndef LCPSOLVER_H
#define LCPSOLVER_H

// Libraries
#include <vector>
#include <map>
#include "../Vector.h"
#include "../Matrix.h"
#include "../../body/Body.h"
#include "../../constants.h"

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
        LCPSolver(uint maxIterations);                                                                                      // Constructor
        virtual ~LCPSolver();                                                                                               // Destructor
        virtual void solve(Matrix1x6** J_sp, Vector6D** B_sp, uint nbConstraints,
                           uint nbBodies, Body*** const bodyMapping, std::map<Body*, uint> bodyNumberMapping,
                           const Vector& b, const Vector& lowLimits, const Vector& highLimits, Vector& lambda) const=0;     // Solve a LCP problem
        void setLambdaInit(const Vector& lambdaInit);                                                                       // Set the initial lambda vector
        void setMaxIterations(uint maxIterations);                                                                          // Set the maximum number of iterations
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
