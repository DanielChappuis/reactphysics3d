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

#ifndef LCP_PROJECTED_GAUSS_SEIDEL_H
#define LCP_PROJECTED_GAUSS_SEIDEL_H

// Libraries
#include "LCPSolver.h"
#include <map>

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

        void computeVectorA(const Vector& lambda, uint nbConstraints, Body*** const bodyMapping,
                            Vector6D** B_sp, std::map<Body*, uint> bodyNumberMapping,
                            Vector6D* const a, uint nbBodies) const ;                                                       // Compute the vector a used in the solve() method

    public:
        LCPProjectedGaussSeidel(uint maxIterations);                                                                    // Constructor
        virtual ~LCPProjectedGaussSeidel();                                                                             // Destructor
        virtual void solve(Matrix1x6** J_sp, Vector6D** B_sp, uint nbConstraints,
                           uint nbBodies, Body*** const bodyMapping, std::map<Body*, uint> bodyNumberMapping,
                           const Vector& b, const Vector& lowLimits, const Vector& highLimits, Vector& lambda) const;   // Solve a LCP problem using Projected-Gauss-Seidel algorithm                                                                                                       // Set the initial value for lambda
};

} // End of the ReactPhysics3D namespace

#endif
