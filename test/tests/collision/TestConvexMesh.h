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

#ifndef TEST_CONVEX_MESH_H
#define TEST_CONVEX_MESH_H

// Libraries
#include "Test.h"
#include <reactphysics3d/reactphysics3d.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestConvexMesh
/**
 * Unit test for the ConvexMesh class.
 */
class TestConvexMesh : public Test {

    private :

        // ---------- Atributes ---------- //

        PhysicsCommon mPhysicsCommon;
        ConvexMesh* mConvexMesh;
        float mVertices[24];
        uint32 mIndices[24];

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestConvexMesh(const std::string& name) : Test(name) {

            // Cube
            mVertices[0] = -3; mVertices[1] = -3; mVertices[2] = 3;
            mVertices[3] = 3; mVertices[4] = -3; mVertices[5] = 3;
            mVertices[6] = 3; mVertices[7] = -3; mVertices[8] = -3;
            mVertices[9] = -3; mVertices[10] = -3; mVertices[11] = -3;
            mVertices[12] = -3; mVertices[13] = 3; mVertices[14] = 3;
            mVertices[15] = 3; mVertices[16] = 3; mVertices[17] = 3;
            mVertices[18] = 3; mVertices[19] = 3; mVertices[20] = -3;
            mVertices[21] = -3; mVertices[22] = 3; mVertices[23] = -3;

            mIndices[0] = 0; mIndices[1] = 3; mIndices[2] = 2; mIndices[3] = 1;
            mIndices[4] = 4; mIndices[5] = 5; mIndices[6] = 6; mIndices[7] = 7;
            mIndices[8] = 0; mIndices[9] = 1; mIndices[10] = 5; mIndices[11] = 4;
            mIndices[12] = 1; mIndices[13] = 2; mIndices[14] = 6; mIndices[15] = 5;
            mIndices[16] = 2; mIndices[17] = 3; mIndices[18] = 7; mIndices[19] = 6;
            mIndices[20] = 0; mIndices[21] = 4; mIndices[22] = 7; mIndices[23] = 3;

            PolygonVertexArray::PolygonFace faces[6];
            rp3d::PolygonVertexArray::PolygonFace* face = faces;
            for (int f = 0; f < 6; f++) {
                face->indexBase = f * 4;
                face->nbVertices = 4;
                face++;
            }
            PolygonVertexArray polygonVertexArray(8, &(mVertices[0]), 3 * sizeof(float),
                    &(mIndices[0]), sizeof(int), 6, faces,
                    rp3d::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                    rp3d::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
            std::vector<Message> messages;
            mConvexMesh = mPhysicsCommon.createConvexMesh(polygonVertexArray, messages);
            for (const auto& m : messages) {
                std::cout << "Message: " << m.text << std::endl;
            }

            rp3d_test(mConvexMesh != nullptr);
        }

        /// Destructor
        virtual ~TestConvexMesh() {

        }

        /// Run the tests
        void run() {
            test();
        }

        void test() {

            // Vertices
            rp3d_test(Vector3::approxEqual(mConvexMesh->getVertex(0), Vector3(mVertices[0], mVertices[1], mVertices[2])));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getVertex(1), Vector3(mVertices[3], mVertices[4], mVertices[5])));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getVertex(2), Vector3(mVertices[6], mVertices[7], mVertices[8])));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getVertex(3), Vector3(mVertices[9], mVertices[10], mVertices[11])));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getVertex(4), Vector3(mVertices[12], mVertices[13], mVertices[14])));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getVertex(5), Vector3(mVertices[15], mVertices[16], mVertices[17])));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getVertex(6), Vector3(mVertices[18], mVertices[19], mVertices[20])));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getVertex(7), Vector3(mVertices[21], mVertices[22], mVertices[23])));

            // Normals
            rp3d_test(Vector3::approxEqual(mConvexMesh->getFaceNormal(0), Vector3(0, -1, 0)));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getFaceNormal(1), Vector3(0, 1, 0)));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getFaceNormal(2), Vector3(0, 0, 1)));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getFaceNormal(3), Vector3(1, 0, 0)));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getFaceNormal(4), Vector3(0, 0, -1)));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getFaceNormal(5), Vector3(-1, 0, 0)));

            rp3d_test(mConvexMesh->getNbVertices() == 8);
            rp3d_test(mConvexMesh->getHalfEdgeStructure().getNbVertices() == 8);
            rp3d_test(mConvexMesh->getNbFaces() == 6);
            rp3d_test(mConvexMesh->getHalfEdgeStructure().getNbFaces() == 6);
            rp3d_test(mConvexMesh->getHalfEdgeStructure().getNbHalfEdges() == 12 * 2);
            rp3d_test(mConvexMesh->getVolume() == 216);

            rp3d_test(Vector3::approxEqual(mConvexMesh->getCentroid(), Vector3::zero()));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getBounds().getMin(), Vector3(-3, -3 ,-3)));
            rp3d_test(Vector3::approxEqual(mConvexMesh->getBounds().getMax(), Vector3(3, 3 ,3)));
        }
 };

}

#endif
