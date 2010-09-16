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
#include "LCPProjectedGaussSeidel.h"
#include <cmath>

using namespace reactphysics3d;
using namespace std;

// Constructor
LCPProjectedGaussSeidel::LCPProjectedGaussSeidel(uint maxIterations)
                        :LCPSolver(maxIterations) {

}

// Destructor
LCPProjectedGaussSeidel::~LCPProjectedGaussSeidel() {

}

// Solve a LCP problem using the Projected-Gauss-Seidel algorithm
// This method outputs the result in the lambda vector
void LCPProjectedGaussSeidel::solve(Matrix1x6** J_sp, Vector6D** B_sp, uint nbConstraints,
                                    uint nbBodies, Body*** const bodyMapping, map<Body*, uint> bodyNumberMapping,
                                    const Vector& b, const Vector& lowLimits, const Vector& highLimits, Vector& lambda) const {

    lambda = lambdaInit;

    double* d = new double[nbConstraints];         // TODO : Avoid those kind of memory allocation here for optimization (allocate once in the object)
    uint indexBody1, indexBody2;
    double deltaLambda;
    double lambdaTemp;
    uint i, iter;
    Vector6D* a = new Vector6D[nbBodies];           // Array that contains nbBodies vector of dimension 6x1

    // Compute the vector a
    computeVectorA(lambda, nbConstraints, bodyMapping, B_sp, bodyNumberMapping, a, nbBodies);

    // For each constraint
    for (i=0; i<nbConstraints; i++) {
        d[i] = (J_sp[i][0] * B_sp[0][i] + J_sp[i][1] * B_sp[1][i]);
    }

    for(iter=0; iter<maxIterations; iter++) {
        for (i=0; i<nbConstraints; i++) {
            indexBody1 = bodyNumberMapping[bodyMapping[i][0]];
            indexBody2 = bodyNumberMapping[bodyMapping[i][1]];
            deltaLambda = (b.getValue(i) - (J_sp[i][0] * a[indexBody1]) - (J_sp[i][1] * a[indexBody2])) / d[i];
            lambdaTemp = lambda.getValue(i);
            lambda.setValue(i, std::max(lowLimits.getValue(i), std::min(lambda.getValue(i) + deltaLambda, highLimits.getValue(i))));
            deltaLambda = lambda.getValue(i) - lambdaTemp;
            a[indexBody1] = a[indexBody1] + (B_sp[0][i] * deltaLambda);
            a[indexBody2] = a[indexBody2] + (B_sp[1][i] * deltaLambda);
        }
    }

    // Clean
    delete[] d;
    delete[] a;
}

// Compute the vector a used in the solve() method
// Note that a = B * lambda
void LCPProjectedGaussSeidel::computeVectorA(const Vector& lambda, uint nbConstraints, Body*** const bodyMapping,
                                             Vector6D** B_sp, map<Body*, uint> bodyNumberMapping,
                                             Vector6D* const a, uint nbBodies) const {
    uint i;
    uint indexBody1, indexBody2;
    
    // Init the vector a with zero values
    for (i=0; i<nbBodies; i++) {
       a[i].initWithValue(0.0);
    }

    for(i=0; i<nbConstraints; i++) {
        indexBody1 = bodyNumberMapping[bodyMapping[i][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[i][1]];
        a[indexBody1] = a[indexBody1] + (B_sp[0][i] * lambda.getValue(i));
        a[indexBody2] = a[indexBody2] + (B_sp[1][i] * lambda.getValue(i));
    }

}
