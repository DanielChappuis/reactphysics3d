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

#ifndef TEST_TRIANGLE_VERTEX_ARRAY_H
#define TEST_TRIANGLE_VERTEX_ARRAY_H

// Libraries
#include <reactphysics3d/reactphysics3d.h>

/// Reactphysics3D namespace
namespace reactphysics3d {


// Class TestTriangleVertexArray
/**
 * Unit test for the TestTriangleArray class.
 */
class TestTriangleVertexArray : public Test {

    private :

        // ---------- Atributes ---------- //

        float mVertices1[4*3];
        double mVertices2[4*3];
        float mNormals2[4*3];
        uint mIndices1[6];
        short mIndices2[6];
        TriangleVertexArray* mTriangleVertexArray1;
        TriangleVertexArray* mTriangleVertexArray2;

        Vector3 mVertex0;
        Vector3 mVertex1;
        Vector3 mVertex2;
        Vector3 mVertex3;
        Vector3 mVertex4;
        Vector3 mVertex5;
        Vector3 mVertex6;
        Vector3 mVertex7;

        Vector3 mNormal0;
        Vector3 mNormal1;
        Vector3 mNormal2;
        Vector3 mNormal3;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestTriangleVertexArray(const std::string& name) : Test(name) {

            mVertex0 = Vector3(0, 0, 4);
            mVertex1 = Vector3(0, 0, -3);
            mVertex2 = Vector3(-2, 0, 0);
            mVertex3 = Vector3(0, -5, 0);

            // Initialize data
            mVertices1[0] = mVertex0.x; mVertices1[1] = mVertex0.y; mVertices1[2] = mVertex0.z;
            mVertices1[3] = mVertex1.x; mVertices1[4] = mVertex1.y; mVertices1[5] = mVertex1.z;
            mVertices1[6] = mVertex2.x; mVertices1[7] = mVertex2.y; mVertices1[8] = mVertex2.z;
            mVertices1[9] = mVertex3.x; mVertices1[10] = mVertex3.y; mVertices1[11] = mVertex3.z;

            mIndices1[0] = 0; mIndices1[1] = 1; mIndices1[2] = 2;
            mIndices1[3] = 0; mIndices1[4] = 3; mIndices1[5] = 1;

            mVertex4 = Vector3(0, 0, 5);
            mVertex5 = Vector3(0, 0, -7);
            mVertex6 = Vector3(-2, 0, 0);
            mVertex7 = Vector3(0, -5, 0);

            mVertices2[0] = static_cast<double>(mVertex4.x); mVertices2[1] = static_cast<double>(mVertex4.y); mVertices2[2] = static_cast<double>(mVertex4.z);
            mVertices2[3] = static_cast<double>(mVertex5.x); mVertices2[4] = static_cast<double>(mVertex5.y); mVertices2[5] = static_cast<double>(mVertex5.z);
            mVertices2[6] = static_cast<double>(mVertex6.x); mVertices2[7] = static_cast<double>(mVertex6.y); mVertices2[8] = static_cast<double>(mVertex6.z);
            mVertices2[9] = static_cast<double>(mVertex7.x); mVertices2[10] = static_cast<double>(mVertex7.y); mVertices2[11] = static_cast<double>(mVertex7.z);

            mIndices2[0] = 0; mIndices2[1] = 1; mIndices2[2] = 2;
            mIndices2[3] = 0; mIndices2[4] = 3; mIndices2[5] = 1;

            mNormal0 = Vector3(2, 4, 6);
            mNormal1 = Vector3(1, 6, -3);
            mNormal2 = Vector3(-2, 4, 7);
            mNormal3 = Vector3(-5, 2, 9);
            mNormal0.normalize();
            mNormal1.normalize();
            mNormal2.normalize();
            mNormal3.normalize();
            mNormals2[0] = mNormal0.x; mNormals2[1] = mNormal0.y; mNormals2[2] = mNormal0.z;
            mNormals2[3] = mNormal1.x; mNormals2[4] = mNormal1.y; mNormals2[5] = mNormal1.z;
            mNormals2[6] = mNormal2.x; mNormals2[7] = mNormal2.y; mNormals2[8] = mNormal2.z;
            mNormals2[9] = mNormal3.x; mNormals2[10] = mNormal3.y; mNormals2[11] = mNormal3.z;

            // Create triangle vertex array with automatic normals computation
            mTriangleVertexArray1 = new TriangleVertexArray(4, static_cast<const void*>(mVertices1), 3 * sizeof(float),
                                                            2, static_cast<const void*>(mIndices1), 3 * sizeof(uint),
                                                            TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                                                            TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE);

            // Create triangle vertex array with normals defined by the user
            mTriangleVertexArray2 = new TriangleVertexArray(4, static_cast<const void*>(mVertices2), 3 * sizeof(double),
                                                            static_cast<const void*>(mNormals2), 3 * sizeof(float),
                                                            2, static_cast<const void*>(mIndices2), 3 * sizeof(short),
                                                            TriangleVertexArray::VertexDataType::VERTEX_DOUBLE_TYPE,
                                                            TriangleVertexArray::NormalDataType::NORMAL_FLOAT_TYPE,
                                                            TriangleVertexArray::IndexDataType::INDEX_SHORT_TYPE);

        }

        /// Destructor
        virtual ~TestTriangleVertexArray() {
            delete mTriangleVertexArray1;
            delete mTriangleVertexArray2;
        }

        /// Run the tests
        void run() {

            // ----- First triangle vertex array ----- //

            rp3d_test(mTriangleVertexArray1->getVertexDataType() == TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE);
            rp3d_test(mTriangleVertexArray1->getIndexDataType() == TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
            rp3d_test(mTriangleVertexArray1->getVertexNormalDataType() == TriangleVertexArray::NormalDataType::NORMAL_FLOAT_TYPE);
            rp3d_test(mTriangleVertexArray1->getNbTriangles() == 2);
            rp3d_test(mTriangleVertexArray1->getNbVertices() == 4);
            rp3d_test(mTriangleVertexArray1->getIndicesStart() == static_cast<const void*>(mIndices1));
            rp3d_test(mTriangleVertexArray1->getVerticesStart() == static_cast<const void*>(mVertices1));
            rp3d_test(mTriangleVertexArray1->getIndicesStride() == (3 * sizeof(uint)));
            rp3d_test(mTriangleVertexArray1->getVerticesStride() == (3 * sizeof(float)));

            // Get triangle indices

            uint triangle0Indices[3];
            mTriangleVertexArray1->getTriangleVerticesIndices(0, triangle0Indices);

            rp3d_test(triangle0Indices[0] == mIndices1[0]);
            rp3d_test(triangle0Indices[1] == mIndices1[1]);
            rp3d_test(triangle0Indices[2] == mIndices1[2]);

            uint triangle1Indices[3];
            mTriangleVertexArray1->getTriangleVerticesIndices(1, triangle1Indices);

            rp3d_test(triangle1Indices[0] == mIndices1[3]);
            rp3d_test(triangle1Indices[1] == mIndices1[4]);
            rp3d_test(triangle1Indices[2] == mIndices1[5]);

            // Get triangle vertices

            Vector3 triangle0Vertices[3];
            mTriangleVertexArray1->getTriangleVertices(0, triangle0Vertices);

            rp3d_test(approxEqual(triangle0Vertices[0], mVertex0, decimal(0.0000001)));
            rp3d_test(approxEqual(triangle0Vertices[1], mVertex1, decimal(0.0000001)));
            rp3d_test(approxEqual(triangle0Vertices[2], mVertex2, decimal(0.0000001)));

            Vector3 triangle1Vertices[3];
            mTriangleVertexArray1->getTriangleVertices(1, triangle1Vertices);

            rp3d_test(approxEqual(triangle1Vertices[0], mVertex0, decimal(0.0000001)));
            rp3d_test(approxEqual(triangle1Vertices[1], mVertex3, decimal(0.0000001)));
            rp3d_test(approxEqual(triangle1Vertices[2], mVertex1, decimal(0.0000001)));

            // Get triangle normals

            Vector3 triangle0Normals[3];
            mTriangleVertexArray1->getTriangleVerticesNormals(0, triangle0Normals);

            Vector3 triangle1Normals[3];
            mTriangleVertexArray1->getTriangleVerticesNormals(1, triangle1Normals);

            const Vector3 normal0Test(decimal(0.9792), decimal(0.20268), 0);
            const Vector3 normal2Test(0, 1, 0);
            const Vector3 normal3Test(1, 0, 0);

            rp3d_test(approxEqual(triangle0Normals[0], normal0Test, decimal(0.0001)));
            rp3d_test(approxEqual(triangle0Normals[2], normal2Test, decimal(0.0001)));
            rp3d_test(approxEqual(triangle1Normals[1], normal3Test, decimal(0.0001)));

            // ----- Second triangle vertex array ----- //

            rp3d_test(mTriangleVertexArray2->getVertexDataType() == TriangleVertexArray::VertexDataType::VERTEX_DOUBLE_TYPE);
            rp3d_test(mTriangleVertexArray2->getIndexDataType() == TriangleVertexArray::IndexDataType::INDEX_SHORT_TYPE);
            rp3d_test(mTriangleVertexArray2->getVertexNormalDataType() == TriangleVertexArray::NormalDataType::NORMAL_FLOAT_TYPE);
            rp3d_test(mTriangleVertexArray2->getNbTriangles() == 2);
            rp3d_test(mTriangleVertexArray2->getNbVertices() == 4);
            rp3d_test(mTriangleVertexArray2->getIndicesStart() == static_cast<const void*>(mIndices2));
            rp3d_test(mTriangleVertexArray2->getVerticesStart() == static_cast<const void*>(mVertices2));
            rp3d_test(mTriangleVertexArray2->getVerticesNormalsStart() == static_cast<const void*>(mNormals2));
            rp3d_test(mTriangleVertexArray2->getIndicesStride() == (3 * sizeof(short)));
            rp3d_test(mTriangleVertexArray2->getVerticesStride() == (3 * sizeof(double)));
            rp3d_test(mTriangleVertexArray2->getVerticesNormalsStride() == (3 * sizeof(float)));

            // Get triangle indices

            mTriangleVertexArray2->getTriangleVerticesIndices(0, triangle0Indices);

            rp3d_test(triangle0Indices[0] == static_cast<uint>(mIndices2[0]));
            rp3d_test(triangle0Indices[1] == static_cast<uint>(mIndices2[1]));
            rp3d_test(triangle0Indices[2] == static_cast<uint>(mIndices2[2]));

            mTriangleVertexArray2->getTriangleVerticesIndices(1, triangle1Indices);

            rp3d_test(triangle1Indices[0] == static_cast<uint>(mIndices2[3]));
            rp3d_test(triangle1Indices[1] == static_cast<uint>(mIndices2[4]));
            rp3d_test(triangle1Indices[2] == static_cast<uint>(mIndices2[5]));

            // Get triangle vertices

            mTriangleVertexArray2->getTriangleVertices(0, triangle0Vertices);

            rp3d_test(approxEqual(triangle0Vertices[0], mVertex4, decimal(0.0000001)));
            rp3d_test(approxEqual(triangle0Vertices[1], mVertex5, decimal(0.0000001)));
            rp3d_test(approxEqual(triangle0Vertices[2], mVertex6, decimal(0.0000001)));

            mTriangleVertexArray2->getTriangleVertices(1, triangle1Vertices);

            rp3d_test(approxEqual(triangle1Vertices[0], mVertex4, decimal(0.0000001)));
            rp3d_test(approxEqual(triangle1Vertices[1], mVertex7, decimal(0.0000001)));
            rp3d_test(approxEqual(triangle1Vertices[2], mVertex5, decimal(0.0000001)));

            // Get triangle normals

            mTriangleVertexArray2->getTriangleVerticesNormals(0, triangle0Normals);
            mTriangleVertexArray2->getTriangleVerticesNormals(1, triangle1Normals);

            rp3d_test(approxEqual(triangle0Normals[0], mNormal0, decimal(0.000001)));
            rp3d_test(approxEqual(triangle0Normals[1], mNormal1, decimal(0.000001)));
            rp3d_test(approxEqual(triangle0Normals[2], mNormal2, decimal(0.000001)));

            rp3d_test(approxEqual(triangle1Normals[0], mNormal0, decimal(0.000001)));
            rp3d_test(approxEqual(triangle1Normals[1], mNormal3, decimal(0.000001)));
            rp3d_test(approxEqual(triangle1Normals[2], mNormal1, decimal(0.000001)));
        }

};

}

#endif

