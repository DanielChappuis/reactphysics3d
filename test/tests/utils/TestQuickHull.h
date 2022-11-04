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
            testSimpleMesh();
        }

        /// Test a simple mesh
        void testSimpleMesh() {

            Array<Vector3> points(mAllocator);

            /*
            points.add(Vector3(1, 4, 8));
            points.add(Vector3(-5, 0, 9));
            points.add(Vector3(-6, 1, 2));
            points.add(Vector3(10, 6, 2));
            points.add(Vector3(6, 6, -5));
            points.add(Vector3(2, -2, -8));
            points.add(Vector3(-4, 8, -1));
            points.add(Vector3(4, 8, 4));
            */

            //
            points.add(Vector3(0, 0, 0));
            points.add(Vector3(4, 0, 0));
            points.add(Vector3(0, 0, 4));
            points.add(Vector3(4, 0, 4));
            points.add(Vector3(0, 4, 0));
            points.add(Vector3(4, 4, 0));
            points.add(Vector3(0, 4, 4));
            points.add(Vector3(4, 4, 4));
            points.add(Vector3(2, 0, 2));

            std::cout << "---------- QuickHull ----------" << std::endl;

            std::cout << "--- Points ---" << std::endl;
            for (uint32 p=0; p < points.size(); p++) {
                std::cout << "Point " << p << ": (" << points[p].x << ", " << points[p].y << ", " << points[p].z << ")" << std::endl;
            }

            VertexArray::DataType dataType = sizeof(decimal) == sizeof(float) ? VertexArray::DataType::VERTEX_FLOAT_TYPE : VertexArray::DataType::VERTEX_DOUBLE_TYPE;
            VertexArray vertexArray(&(points[0]), sizeof(Vector3), points.size(), dataType);

            std::vector<Error> errors;
            ConvexMesh* mesh = mPhysicsCommon.createConvexMesh(vertexArray, errors);
            rp3d_test(mesh != nullptr);

            Array<uint32> indicesOfHull(mAllocator);
            indicesOfHull.add(0);
            indicesOfHull.add(1);
            indicesOfHull.add(2);
            indicesOfHull.add(3);
            indicesOfHull.add(4);
            indicesOfHull.add(5);
            indicesOfHull.add(6);
            indicesOfHull.add(7);
            //rp3d_test(testPointsAmongHullVertices(points, indicesOfHull, mesh));

            /*
            for (uint32 f=0; f < polygonVertexArray->getNbFaces(); f++) {

                std::cout << " :: Face " << f << " ::" << std::endl;
                PolygonVertexArray::PolygonFace* face = polygonVertexArray->getPolygonFace(f);
                for (uint32 v=0; v < face->nbVertices; v++) {

                    uint32 vertexIndex = polygonVertexArray->getVertexIndexInFace(f, v);
                    Vector3 vertex = polygonVertexArray->getVertex(vertexIndex);

                    std::cout << "   :: Vertex " << vertexIndex << ": (" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << std::endl;
                }
            }
            */
        }

        bool testPointsAmongHullVertices(const Array<Vector3>& points, const Array<uint32>& indicesOfHull,  PolygonVertexArray* polygonVertexArray) {

            bool isValid = true;

            // For each point that should be in the convex hull
            for(uint32 i=0; i < indicesOfHull.size(); i++) {
                isValid &= testPointAmongHullVertices(points[indicesOfHull[i]], polygonVertexArray);
            }

            return isValid;
        }

        bool testPointAmongHullVertices(const Vector3& vertex, PolygonVertexArray* polygonVertexArray) {

            for (uint32 i=0; i < polygonVertexArray->getNbVertices(); i++) {
                Vector3 point = polygonVertexArray->getVertex(i);

                if (point == vertex) {
                    return true;
                }
            }

            return false;
        }
 };

}

#endif
