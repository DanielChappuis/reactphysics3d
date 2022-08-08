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
        

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestQuickHull(const std::string& name) : Test(name) {

        }

        /// Run the tests
        void run() {
            testSimpleMesh();
        }

        /// Test a simple mesh
        void testSimpleMesh() {

            Array<Vector3> points(mAllocator);
            points.add(Vector3(1, 4, 8));
            points.add(Vector3(-5, 0, 9));
            points.add(Vector3(-6, 1, 2));
            points.add(Vector3(10, 6, 2));
            points.add(Vector3(6, 6, -5));
            points.add(Vector3(2, -2, -8));
            points.add(Vector3(-4, 8, -1));
            points.add(Vector3(4, 8, 4));

            std::cout << "---------- QuickHull ----------" << std::endl;

            std::cout << "--- Points ---" << std::endl;
            for (uint32 p=0; p < points.size(); p++) {
                std::cout << "Point " << p << ": (" << points[p].x << ", " << points[p].y << ", " << points[p].z << ")" << std::endl;
            }

            PolygonVertexArray::VertexDataType vertexDataType = sizeof(decimal) == sizeof(float) ? PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE : PolygonVertexArray::VertexDataType::VERTEX_DOUBLE_TYPE;
            PolygonVertexArray* polygonVertexArray = QuickHull::computeConvexHull(points.size(), &(points[0]), sizeof(Vector3), vertexDataType, mAllocator);

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
 };

}

#endif
