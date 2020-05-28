#ifndef TEST_HALF_EDGE_STRUCTURE_H
#define TEST_HALF_EDGE_STRUCTURE_H

// Libraries
#include <reactphysics3d/reactphysics3d.h>
#include "Test.h"
#include <vector>

/// Reactphysics3D namespace
namespace reactphysics3d {


// Class TestHalfEdgeStructure
/**
 * Unit test for the HalfEdgeStructure class.
 */
class TestHalfEdgeStructure : public Test {

    private :

        // ---------- Atributes ---------- //

        /// Memory allocator
        DefaultAllocator mAllocator;


    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestHalfEdgeStructure(const std::string& name) : Test(name) {

        }

        /// Destructor
        virtual ~TestHalfEdgeStructure() {

        }

        /// Run the tests
        void run() {
            testCube();
            testTetrahedron();
        }

        void testCube() {

            // Create the half-edge structure for a cube
            rp3d::HalfEdgeStructure cubeStructure(mAllocator, 6, 8, 24);

            rp3d::Vector3 vertices[8] = {
                rp3d::Vector3(-0.5, -0.5, 0.5),
                rp3d::Vector3(0.5, -0.5, 0.5),
                rp3d::Vector3(0.5, 0.5, 0.5),
                rp3d::Vector3(-0.5, 0.5, 0.5),
                rp3d::Vector3(-0.5, -0.5, -0.5),
                rp3d::Vector3(0.5, -0.5, -0.5),
                rp3d::Vector3(0.5, 0.5, -0.5),
                rp3d::Vector3(-0.5, 0.5, -0.5)
            };

            // Vertices
            cubeStructure.addVertex(0);
            cubeStructure.addVertex(1);
            cubeStructure.addVertex(2);
            cubeStructure.addVertex(3);
            cubeStructure.addVertex(4);
            cubeStructure.addVertex(5);
            cubeStructure.addVertex(6);
            cubeStructure.addVertex(7);

            // Faces
            List<uint> face0(mAllocator, 4);
            face0.add(0); face0.add(1); face0.add(2); face0.add(3);
            List<uint> face1(mAllocator, 4);
            face1.add(1); face1.add(5); face1.add(6); face1.add(2);
            List<uint> face2(mAllocator, 4);
            face2.add(5); face2.add(4); face2.add(7); face2.add(6);
            List<uint> face3(mAllocator, 4);
            face3.add(4); face3.add(0); face3.add(3); face3.add(7);
            List<uint> face4(mAllocator, 4);
            face4.add(0); face4.add(4); face4.add(5); face4.add(1);
            List<uint> face5(mAllocator, 4);
            face5.add(2); face5.add(6); face5.add(7); face5.add(3);

            cubeStructure.addFace(face0);
            cubeStructure.addFace(face1);
            cubeStructure.addFace(face2);
            cubeStructure.addFace(face3);
            cubeStructure.addFace(face4);
            cubeStructure.addFace(face5);

            cubeStructure.init();

            // --- Test that the half-edge structure of the cube is valid --- //

            rp3d_test(cubeStructure.getNbFaces() == 6);
            rp3d_test(cubeStructure.getNbVertices() == 8);
            rp3d_test(cubeStructure.getNbHalfEdges() == 24);

            // Test vertices
            rp3d_test(vertices[cubeStructure.getVertex(0).vertexPointIndex].x == -0.5);
            rp3d_test(vertices[cubeStructure.getVertex(0).vertexPointIndex].y == -0.5);
            rp3d_test(vertices[cubeStructure.getVertex(0).vertexPointIndex].z == 0.5);
            rp3d_test(cubeStructure.getHalfEdge(cubeStructure.getVertex(0).edgeIndex).vertexIndex == 0);

            rp3d_test(vertices[cubeStructure.getVertex(1).vertexPointIndex].x == 0.5);
            rp3d_test(vertices[cubeStructure.getVertex(1).vertexPointIndex].y == -0.5);
            rp3d_test(vertices[cubeStructure.getVertex(1).vertexPointIndex].z == 0.5);
            rp3d_test(cubeStructure.getHalfEdge(cubeStructure.getVertex(1).edgeIndex).vertexIndex == 1);

            rp3d_test(vertices[cubeStructure.getVertex(2).vertexPointIndex].x == 0.5);
            rp3d_test(vertices[cubeStructure.getVertex(2).vertexPointIndex].y == 0.5);
            rp3d_test(vertices[cubeStructure.getVertex(2).vertexPointIndex].z == 0.5);
            rp3d_test(cubeStructure.getHalfEdge(cubeStructure.getVertex(2).edgeIndex).vertexIndex == 2);

            rp3d_test(vertices[cubeStructure.getVertex(3).vertexPointIndex].x == -0.5);
            rp3d_test(vertices[cubeStructure.getVertex(3).vertexPointIndex].y == 0.5);
            rp3d_test(vertices[cubeStructure.getVertex(3).vertexPointIndex].z == 0.5);
            rp3d_test(cubeStructure.getHalfEdge(cubeStructure.getVertex(3).edgeIndex).vertexIndex == 3);

            rp3d_test(vertices[cubeStructure.getVertex(4).vertexPointIndex].x == -0.5);
            rp3d_test(vertices[cubeStructure.getVertex(4).vertexPointIndex].y == -0.5);
            rp3d_test(vertices[cubeStructure.getVertex(4).vertexPointIndex].z == -0.5);
            rp3d_test(cubeStructure.getHalfEdge(cubeStructure.getVertex(4).edgeIndex).vertexIndex == 4);

            rp3d_test(vertices[cubeStructure.getVertex(5).vertexPointIndex].x == 0.5);
            rp3d_test(vertices[cubeStructure.getVertex(5).vertexPointIndex].y == -0.5);
            rp3d_test(vertices[cubeStructure.getVertex(5).vertexPointIndex].z == -0.5);
            rp3d_test(cubeStructure.getHalfEdge(cubeStructure.getVertex(5).edgeIndex).vertexIndex == 5);

            rp3d_test(vertices[cubeStructure.getVertex(6).vertexPointIndex].x == 0.5);
            rp3d_test(vertices[cubeStructure.getVertex(6).vertexPointIndex].y == 0.5);
            rp3d_test(vertices[cubeStructure.getVertex(6).vertexPointIndex].z == -0.5);
            rp3d_test(cubeStructure.getHalfEdge(cubeStructure.getVertex(6).edgeIndex).vertexIndex == 6);

            rp3d_test(vertices[cubeStructure.getVertex(7).vertexPointIndex].x == -0.5);
            rp3d_test(vertices[cubeStructure.getVertex(7).vertexPointIndex].y == 0.5);
            rp3d_test(vertices[cubeStructure.getVertex(7).vertexPointIndex].z == -0.5);
            rp3d_test(cubeStructure.getHalfEdge(cubeStructure.getVertex(7).edgeIndex).vertexIndex == 7);

            // Test faces
            for (uint f=0; f<6; f++) {
                rp3d_test(cubeStructure.getHalfEdge(cubeStructure.getFace(f).edgeIndex).faceIndex == f);
            }

            // Test edges
            for (uint f=0; f<6; f++) {


                uint edgeIndex = cubeStructure.getFace(f).edgeIndex;
                const uint firstEdgeIndex = edgeIndex;

                // For each half-edge of the face
                for (uint e=0; e<4; e++) {

                    rp3d::HalfEdgeStructure::Edge edge = cubeStructure.getHalfEdge(edgeIndex);

                    rp3d_test(cubeStructure.getHalfEdge(edge.twinEdgeIndex).twinEdgeIndex == edgeIndex);
                    rp3d_test(edge.faceIndex == f);

                    // Go to the next edge
                    edgeIndex = edge.nextEdgeIndex;
                }

                rp3d_test(firstEdgeIndex == edgeIndex);
            }
        }

        void testTetrahedron() {

            // Create the half-edge structure for a tetrahedron
            std::vector<std::vector<uint>> faces;
            rp3d::HalfEdgeStructure tetrahedron(mAllocator, 4, 4, 12);

            // Vertices
            rp3d::Vector3 vertices[4] = {
                rp3d::Vector3(1, -1, -1),
                rp3d::Vector3(-1, -1, -1),
                rp3d::Vector3(0, -1, 1),
                rp3d::Vector3(0, 1, 0)
            };

            tetrahedron.addVertex(0);
            tetrahedron.addVertex(1);
            tetrahedron.addVertex(2);
            tetrahedron.addVertex(3);

            // Faces
            List<uint> face0(mAllocator, 3);
            face0.add(0); face0.add(1); face0.add(2);
            List<uint> face1(mAllocator, 3);
            face1.add(0); face1.add(3); face1.add(1);
            List<uint> face2(mAllocator, 3);
            face2.add(1); face2.add(3); face2.add(2);
            List<uint> face3(mAllocator, 3);
            face3.add(0); face3.add(2); face3.add(3);

            tetrahedron.addFace(face0);
            tetrahedron.addFace(face1);
            tetrahedron.addFace(face2);
            tetrahedron.addFace(face3);

            tetrahedron.init();

            // --- Test that the half-edge structure of the tetrahedron is valid --- //

            rp3d_test(tetrahedron.getNbFaces() == 4);
            rp3d_test(tetrahedron.getNbVertices() == 4);
            rp3d_test(tetrahedron.getNbHalfEdges() == 12);

            // Test vertices
            rp3d_test(vertices[tetrahedron.getVertex(0).vertexPointIndex].x == 1);
            rp3d_test(vertices[tetrahedron.getVertex(0).vertexPointIndex].y == -1);
            rp3d_test(vertices[tetrahedron.getVertex(0).vertexPointIndex].z == -1);
            rp3d_test(tetrahedron.getHalfEdge(tetrahedron.getVertex(0).edgeIndex).vertexIndex == 0);

            rp3d_test(vertices[tetrahedron.getVertex(1).vertexPointIndex].x == -1);
            rp3d_test(vertices[tetrahedron.getVertex(1).vertexPointIndex].y == -1);
            rp3d_test(vertices[tetrahedron.getVertex(1).vertexPointIndex].z == -1);
            rp3d_test(tetrahedron.getHalfEdge(tetrahedron.getVertex(1).edgeIndex).vertexIndex == 1);

            rp3d_test(vertices[tetrahedron.getVertex(2).vertexPointIndex].x == 0);
            rp3d_test(vertices[tetrahedron.getVertex(2).vertexPointIndex].y == -1);
            rp3d_test(vertices[tetrahedron.getVertex(2).vertexPointIndex].z == 1);
            rp3d_test(tetrahedron.getHalfEdge(tetrahedron.getVertex(2).edgeIndex).vertexIndex == 2);

            rp3d_test(vertices[tetrahedron.getVertex(3).vertexPointIndex].x == 0);
            rp3d_test(vertices[tetrahedron.getVertex(3).vertexPointIndex].y == 1);
            rp3d_test(vertices[tetrahedron.getVertex(3).vertexPointIndex].z == 0);
            rp3d_test(tetrahedron.getHalfEdge(tetrahedron.getVertex(3).edgeIndex).vertexIndex == 3);

            // Test faces
            for (uint f=0; f<4; f++) {
                rp3d_test(tetrahedron.getHalfEdge(tetrahedron.getFace(f).edgeIndex).faceIndex == f);
            }

            // Test edges
            for (uint f=0; f<4; f++) {

                uint edgeIndex = tetrahedron.getFace(f).edgeIndex;
                const uint firstEdgeIndex = edgeIndex;

                // For each half-edge of the face
                for (uint e=0; e<3; e++) {

                    rp3d::HalfEdgeStructure::Edge edge = tetrahedron.getHalfEdge(edgeIndex);

                    rp3d_test(tetrahedron.getHalfEdge(edge.twinEdgeIndex).twinEdgeIndex == edgeIndex);
                    rp3d_test(edge.faceIndex == f);

                    // Go to the next edge
                    edgeIndex = edge.nextEdgeIndex;
                }

                rp3d_test(firstEdgeIndex == edgeIndex);
            }
        }
 };

}

#endif
