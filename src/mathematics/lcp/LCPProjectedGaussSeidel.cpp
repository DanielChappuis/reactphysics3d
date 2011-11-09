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
void LCPProjectedGaussSeidel::solve(double J_sp[NB_MAX_CONSTRAINTS][2*6], double B_sp[2][6*NB_MAX_CONSTRAINTS], uint nbConstraints,
                                    uint nbBodies, Body* bodyMapping[NB_MAX_CONSTRAINTS][2], map<Body*, uint> bodyNumberMapping,
                                    double b[], double lowLimits[NB_MAX_CONSTRAINTS], double highLimits[NB_MAX_CONSTRAINTS], double lambda[NB_MAX_CONSTRAINTS]) const {

	for (uint i=0; i<nbConstraints; i++) {
		lambda[i] = lambdaInit[i];
	}


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
		uint indexConstraintArray = 6 * i;
		d[i] = 0.0;
		for (uint j=0; j<6; j++) {
			d[i] += J_sp[i][j] * B_sp[0][indexConstraintArray + j] + J_sp[i][6 + j] * B_sp[1][indexConstraintArray + j];
		}
    }


    for(iter=0; iter<maxIterations; iter++) {
        for (i=0; i<nbConstraints; i++) {
            indexBody1 = bodyNumberMapping[bodyMapping[i][0]];
            indexBody2 = bodyNumberMapping[bodyMapping[i][1]];
			uint indexConstraintArray = 6 * i;
			deltaLambda = b[i];
			for (uint j=0; j<6; j++) {
				deltaLambda -= (J_sp[i][j] * a[indexBody1].getValue(j) + J_sp[i][6 + j] * a[indexBody2].getValue(j));
			}
			deltaLambda /= d[i];
            lambdaTemp = lambda[i];
            lambda[i] = std::max(lowLimits[i], std::min(lambda[i] + deltaLambda, highLimits[i]));
            deltaLambda = lambda[i] - lambdaTemp;
			for (uint j=0; j<6; j++) {
				a[indexBody1].setValue(j, a[indexBody1].getValue(j) + (B_sp[0][indexConstraintArray + j] * deltaLambda));
				a[indexBody2].setValue(j, a[indexBody2].getValue(j) + (B_sp[1][indexConstraintArray + j] * deltaLambda));
			}
        }
    }


    // Clean
    delete[] d;
    delete[] a;
}

// Compute the vector a used in the solve() method
// Note that a = B * lambda
void LCPProjectedGaussSeidel::computeVectorA(double lambda[NB_MAX_CONSTRAINTS], uint nbConstraints, Body* bodyMapping[NB_MAX_CONSTRAINTS][2],
                                             double B_sp[2][6*NB_MAX_CONSTRAINTS], map<Body*, uint> bodyNumberMapping,
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
		uint indexConstraintArray = 6 * i;
		for (uint j=0; j<6; j++) {
			a[indexBody1].setValue(j, a[indexBody1].getValue(j) + (B_sp[0][indexConstraintArray + j] * lambda[i]));
			a[indexBody2].setValue(j, a[indexBody2].getValue(j) + (B_sp[1][indexConstraintArray + j] * lambda[i]));
		}
    }
}
