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
#include <iostream>
#include "TestSuite/Suite.h"
#include "testing_mathematics/MathematicsTest.h"
#include "testing_mathematics/VectorTest.h"
#include "testing_mathematics/Vector3DTest.h"
#include "testing_mathematics/MatrixTest.h"
#include "testing_mathematics/Matrix3x3Test.h"
#include "testing_mathematics/QuaternionTest.h"
#include "testing_physics/TimeTest.h"
#include "testing_physics/KilogramTest.h"

// Namespaces
using namespace std;
using namespace TestSuite;

// Main function
int main() {

    // ReactPhysics3D TestSuite
    Suite reactphysics3DTestSuite("ReactPhysics3D TestSuite");

    // Mathematics tests
    reactphysics3DTestSuite.addTest(new MathematicsTest);
    reactphysics3DTestSuite.addTest(new VectorTest);
    reactphysics3DTestSuite.addTest(new Vector3DTest);
    reactphysics3DTestSuite.addTest(new MatrixTest);
    reactphysics3DTestSuite.addTest(new Matrix3x3Test);
    reactphysics3DTestSuite.addTest(new QuaternionTest);

    // Physics tests
    reactphysics3DTestSuite.addTest(new TimeTest);
    reactphysics3DTestSuite.addTest(new KilogramTest);

    // Run the ReactPhysics3D TestSuite and display the report
    reactphysics3DTestSuite.run();
    long nbFailures = reactphysics3DTestSuite.report();
    reactphysics3DTestSuite.free();
    return nbFailures;
    double inPause;
    cin >> inPause;
}
