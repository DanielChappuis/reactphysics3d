/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef TEST_QUICKHULL_H
#define TEST_QUICKHULL_H

// Libraries
#include "Test.h"
#include <reactphysics3d/utils/quickhull/QuickHull.h>
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/engine/PhysicsCommon.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestQuickHull
/**
 * Unit test for the Quick-Hull algorithm
 */
class TestQuickHull : public Test {

    private :

        // ---------- Atributes ---------- //
    
        DefaultAllocator mAllocator;
        PhysicsCommon mPhysicsCommon;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestQuickHull(const std::string& name) : Test(name) {

        }

        /// Destructor
        virtual ~TestQuickHull() = default;

        /// Run the tests
        void run() {
            testConvexHulls();
        }

        /// Test a simple mesh
        void testConvexHulls() {

            std::vector<Vector3> points;

            points.push_back(Vector3(1, 4, 8));
            points.push_back(Vector3(-5, 0, 9));
            points.push_back(Vector3(-6, 1, 2));
            points.push_back(Vector3(10, 6, 2));
            points.push_back(Vector3(6, 6, -5));
            points.push_back(Vector3(2, -2, -8));
            points.push_back(Vector3(-4, 8, -1));
            points.push_back(Vector3(4, 8, 4));

            VertexArray::DataType dataType = sizeof(decimal) == sizeof(float) ? VertexArray::DataType::VERTEX_FLOAT_TYPE : VertexArray::DataType::VERTEX_DOUBLE_TYPE;
            VertexArray vertexArray1(points.data(), sizeof(Vector3), points.size(), dataType);

            std::vector<Message> errors;
            ConvexMesh* mesh = mPhysicsCommon.createConvexMesh(vertexArray1, errors);
            rp3d_test(mesh != nullptr);

            std::vector<uint32> indicesOfHull;
            indicesOfHull.push_back(0);
            indicesOfHull.push_back(1);
            indicesOfHull.push_back(2);
            indicesOfHull.push_back(3);
            indicesOfHull.push_back(4);
            indicesOfHull.push_back(5);
            indicesOfHull.push_back(6);
            indicesOfHull.push_back(7);
            rp3d_test(testPointsAmongHullVertices(points, indicesOfHull, mesh));

            // Cube
            points.clear();
            points.push_back(Vector3(0, 0, 0));
            points.push_back(Vector3(4, 0, 0));
            points.push_back(Vector3(0, 0, 4));
            points.push_back(Vector3(4, 0, 4));
            points.push_back(Vector3(0, 4, 0));
            points.push_back(Vector3(4, 4, 0));
            points.push_back(Vector3(0, 4, 4));
            points.push_back(Vector3(4, 4, 4));
            points.push_back(Vector3(2, 0, 2));
            points.push_back(Vector3(0, 2, 2));
            points.push_back(Vector3(2, 2, 0));
            points.push_back(Vector3(4, 2, 2));
            points.push_back(Vector3(2, 2, 4));
            points.push_back(Vector3(2, 4, 2));
            points.push_back(Vector3(2, 2, 2));

            dataType = sizeof(decimal) == sizeof(float) ? VertexArray::DataType::VERTEX_FLOAT_TYPE : VertexArray::DataType::VERTEX_DOUBLE_TYPE;
            VertexArray vertexArray2(points.data(), sizeof(Vector3), points.size(), dataType);

            errors.clear();
            mesh = mPhysicsCommon.createConvexMesh(vertexArray2, errors);
            rp3d_test(mesh != nullptr);

            indicesOfHull.clear();
            indicesOfHull.push_back(0);
            indicesOfHull.push_back(1);
            indicesOfHull.push_back(2);
            indicesOfHull.push_back(3);
            indicesOfHull.push_back(4);
            indicesOfHull.push_back(5);
            indicesOfHull.push_back(6);
            indicesOfHull.push_back(7);
            rp3d_test(testPointsAmongHullVertices(points, indicesOfHull, mesh));

            points.clear();
            points.push_back(Vector3(10, 11, 56));
            points.push_back(Vector3(-4, -45, -34));
            points.push_back(Vector3(2, -99, 95));
            points.push_back(Vector3(23, 21, -25));
            points.push_back(Vector3(-56, 45, 75));
            points.push_back(Vector3(29, -87, 12));
            points.push_back(Vector3(-12, -1, 98));
            points.push_back(Vector3(-48, 59, -12));
            points.push_back(Vector3(2, -21, -28));
            points.push_back(Vector3(0, -82, 48));
            points.push_back(Vector3(23, 91, -5));
            points.push_back(Vector3(-64, 24, 0));

            dataType = sizeof(decimal) == sizeof(float) ? VertexArray::DataType::VERTEX_FLOAT_TYPE : VertexArray::DataType::VERTEX_DOUBLE_TYPE;
            VertexArray vertexArray3(points.data(), sizeof(Vector3), points.size(), dataType);

            errors.clear();
            mesh = mPhysicsCommon.createConvexMesh(vertexArray3, errors);
            rp3d_test(mesh != nullptr);

            indicesOfHull.clear();
            indicesOfHull.push_back(0);
            indicesOfHull.push_back(1);
            indicesOfHull.push_back(2);
            indicesOfHull.push_back(3);
            indicesOfHull.push_back(4);
            indicesOfHull.push_back(5);
            indicesOfHull.push_back(6);
            indicesOfHull.push_back(7);
            indicesOfHull.push_back(9);
            indicesOfHull.push_back(10);
            indicesOfHull.push_back(11);
            rp3d_test(testPointsAmongHullVertices(points, indicesOfHull, mesh));

            points.clear();
            points.push_back(Vector3(3, -45, -145));
            points.push_back(Vector3(-6, 0, 222));
            points.push_back(Vector3(76, -21, -83));
            points.push_back(Vector3(19, -99, -56));
            points.push_back(Vector3(-43, -37, 87));
            points.push_back(Vector3(0, 2, -345));
            points.push_back(Vector3(-15, 49, 1));
            points.push_back(Vector3(91, 82, -38));
            points.push_back(Vector3(-4, -17, -9));
            points.push_back(Vector3(62, -65, -61));
            points.push_back(Vector3(49, 40, 21));
            points.push_back(Vector3(7, 49, 124));
            points.push_back(Vector3(200, -1, 0));

            dataType = sizeof(decimal) == sizeof(float) ? VertexArray::DataType::VERTEX_FLOAT_TYPE : VertexArray::DataType::VERTEX_DOUBLE_TYPE;
            VertexArray vertexArray4(points.data(), sizeof(Vector3), points.size(), dataType);

            errors.clear();
            mesh = mPhysicsCommon.createConvexMesh(vertexArray4, errors);
            rp3d_test(mesh != nullptr);

            indicesOfHull.clear();
            indicesOfHull.push_back(1);
            indicesOfHull.push_back(3);
            indicesOfHull.push_back(4);
            indicesOfHull.push_back(5);
            indicesOfHull.push_back(6);
            indicesOfHull.push_back(7);
            indicesOfHull.push_back(11);
            indicesOfHull.push_back(12);
            rp3d_test(testPointsAmongHullVertices(points, indicesOfHull, mesh));

            points.clear();
            points.push_back(Vector3(34, 0, -58));
            points.push_back(Vector3(5, 3, 31));
            points.push_back(Vector3(87, -76, 0));
            points.push_back(Vector3(-23, -21, 0));
            points.push_back(Vector3(9, -20, -48));
            points.push_back(Vector3(5, 3, 31));

            dataType = sizeof(decimal) == sizeof(float) ? VertexArray::DataType::VERTEX_FLOAT_TYPE : VertexArray::DataType::VERTEX_DOUBLE_TYPE;
            VertexArray vertexArray5(points.data(), sizeof(Vector3), points.size(), dataType);

            errors.clear();
            mesh = mPhysicsCommon.createConvexMesh(vertexArray5, errors);
            rp3d_test(mesh != nullptr);

            indicesOfHull.clear();
            indicesOfHull.push_back(0);
            indicesOfHull.push_back(1);
            indicesOfHull.push_back(2);
            indicesOfHull.push_back(3);
            indicesOfHull.push_back(4);
            rp3d_test(testPointsAmongHullVertices(points, indicesOfHull, mesh));
        }

        bool testPointsAmongHullVertices(const std::vector<Vector3>& points, const std::vector<uint32>& indicesOfHull, ConvexMesh* convexMesh) {

            bool isValid = true;

            // For each point that should be in the convex hull
            for(uint32 index: indicesOfHull) {
                isValid &= testPointAmongHullVertices(points[index], convexMesh);
            }

            return isValid;
        }

        bool testPointAmongHullVertices(const Vector3& vertex, ConvexMesh* convexMesh) {

            for (uint32 i=0; i < convexMesh->getNbVertices(); i++) {
                const Vector3& point = convexMesh->getVertex(i);

                if (point == vertex) {
                    return true;
                }
            }

            return false;
        }
 };

}

#endif
