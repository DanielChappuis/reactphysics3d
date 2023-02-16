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

#ifndef TEST_TRIANGLE_MESH_H
#define TEST_TRIANGLE_MESH_H

// Libraries
#include "Test.h"
#include <reactphysics3d/reactphysics3d.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestTriangleMesh
/**
 * Unit test for the TriangleMesh class.
 */
class TestTriangleMesh : public Test {

    private :

        // ---------- Atributes ---------- //

        PhysicsCommon mPhysicsCommon;
        TriangleMesh* mTriangleMesh;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestTriangleMesh(const std::string& name) : Test(name) {

            float planeVertices[36 * 3];
            int planeIndices[25 * 2 * 3];

            // ---------- Concave Mesh ---------- //
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 6; j++) {
                    planeVertices[i * 6 * 3 + j * 3] = -2.5f + i;
                    planeVertices[i * 6 * 3 + (j * 3) + 1] = 0;
                    planeVertices[i * 6 * 3 + (j * 3) + 2] = -2.5f + j;
                }
            }
            int triangleIndex = 0;
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {

                    // Triangle 1
                    planeIndices[triangleIndex * 3] = i * 6 + j;
                    planeIndices[triangleIndex * 3 + 1] = i * 6+ (j+1);
                    planeIndices[triangleIndex * 3 + 2] = (i+1) * 6 + (j+1);
                    triangleIndex++;

                    // Triangle 2
                    planeIndices[triangleIndex * 3] = i * 6+ j;
                    planeIndices[triangleIndex * 3 + 1] = (i+1) * 6 + (j+1);
                    planeIndices[triangleIndex * 3 + 2] = (i+1) * 6 + j;
                    triangleIndex++;
                }
            }

            rp3d::TriangleVertexArray triangleVertexArray(36, &(planeVertices[0]), 3 * sizeof(float),
                    50, &(planeIndices[0]), 3 * sizeof(int),
                    rp3d::TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                    rp3d::TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE);

            // Add the triangle vertex array of the subpart to the triangle mesh
            std::vector<rp3d::Message> messages;
            mTriangleMesh = mPhysicsCommon.createTriangleMesh(triangleVertexArray, messages);
        }

        /// Destructor
        virtual ~TestTriangleMesh() {

        }

        /// Run the tests
        void run() {
            test();
        }

        void test() {

            // Vertices
            rp3d_test(Vector3::approxEqual(mTriangleMesh->getVertex(0), Vector3(-2.5, 0, -2.5)));
            rp3d_test(Vector3::approxEqual(mTriangleMesh->getVertex(7), Vector3(-1.5, 0, -1.5)));
            rp3d_test(Vector3::approxEqual(mTriangleMesh->getVertex(1), Vector3(-2.5, 0, -1.5)));

            // Normals
            rp3d_test(Vector3::approxEqual(mTriangleMesh->getVertexNormal(0), Vector3(0, 1, 0)));
            rp3d_test(Vector3::approxEqual(mTriangleMesh->getVertexNormal(1), Vector3(0, 1, 0)));

            uint32 v1Index, v2Index, v3Index;
            mTriangleMesh->getTriangleVerticesIndices(0, v1Index, v2Index, v3Index);
            rp3d_test(v1Index == 0);
            rp3d_test(v2Index == 1);
            rp3d_test(v3Index == 7);

            /// Return the coordinates of the three vertices of a given triangle face
            Vector3 v1, v2, v3;
            mTriangleMesh->getTriangleVertices(0, v1, v2, v3);
            rp3d_test(v1 == Vector3(-2.5, 0, -2.5));
            rp3d_test(v2 == Vector3(-2.5, 0, -1.5));
            rp3d_test(v3 == Vector3(-1.5, 0, -1.5));

            /// Return the normals of the three vertices of a given triangle face
            Vector3 n1, n2, n3;
            mTriangleMesh->getTriangleVerticesNormals(0, n1, n2, n3);
            rp3d_test(n1 == Vector3(0, 1, 0));
            rp3d_test(n2 == Vector3(0, 1, 0));
            rp3d_test(n3 == Vector3(0, 1, 0));

            rp3d_test(mTriangleMesh->getNbVertices() == 36);
            rp3d_test(mTriangleMesh->getNbTriangles() == 50);

            rp3d_test(Vector3::approxEqual(mTriangleMesh->getBounds().getMin(), Vector3(-2.5, 0 ,-2.5)));
            rp3d_test(Vector3::approxEqual(mTriangleMesh->getBounds().getMax(), Vector3(2.5, 0, 2.5)));
        }
 };

}

#endif
