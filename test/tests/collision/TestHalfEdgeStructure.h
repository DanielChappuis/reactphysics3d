#ifndef TEST_HALF_EDGE_STRUCTURE_H
#define TEST_HALF_EDGE_STRUCTURE_H

// Libraries
#include "reactphysics3d.h"
#include "Test.h"

/// Reactphysics3D namespace
namespace reactphysics3d {


// Class TestHalfEdgeStructure
/**
 * Unit test for the HalfEdgeStructure class.
 */
class TestHalfEdgeStructure : public Test {

    private :

        // ---------- Atributes ---------- //


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
            std::vector<rp3d::Vector3> vertices;
            std::vector<std::vector<uint>> faces;
            rp3d::HalfEdgeStructure cubeStructure;

            // Vertices
            vertices.push_back(rp3d::Vector3(-0.5, -0.5, 0.5));
            vertices.push_back(rp3d::Vector3(0.5, -0.5, 0.5));
            vertices.push_back(rp3d::Vector3(0.5, 0.5, 0.5));
            vertices.push_back(rp3d::Vector3(-0.5, 0.5, 0.5));
            vertices.push_back(rp3d::Vector3(-0.5, -0.5, -0.5));
            vertices.push_back(rp3d::Vector3(0.5, -0.5, -0.5));
            vertices.push_back(rp3d::Vector3(0.5, 0.5, -0.5));
            vertices.push_back(rp3d::Vector3(-0.5, 0.5, -0.5));

            // Faces
            std::vector<uint> face0;
            face0.push_back(0); face0.push_back(1); face0.push_back(2); face0.push_back(3);
            std::vector<uint> face1;
            face1.push_back(1); face1.push_back(5); face1.push_back(6); face1.push_back(2);
            std::vector<uint> face2;
            face2.push_back(5); face2.push_back(4); face2.push_back(7); face2.push_back(6);
            std::vector<uint> face3;
            face3.push_back(4); face3.push_back(0); face3.push_back(3); face3.push_back(7);
            std::vector<uint> face4;
            face4.push_back(0); face4.push_back(4); face4.push_back(5); face4.push_back(1);
            std::vector<uint> face5;
            face5.push_back(2); face5.push_back(6); face5.push_back(7); face5.push_back(3);

            faces.push_back(face0);
            faces.push_back(face1);
            faces.push_back(face2);
            faces.push_back(face3);
            faces.push_back(face4);
            faces.push_back(face5);

            cubeStructure.init(vertices, faces);

            // --- Test that the half-edge structure of the cube is valid --- //

            test(cubeStructure.getNbFaces() == 6);
            test(cubeStructure.getNbVertices() == 8);
            test(cubeStructure.getNbHalfEdges() == 24);

            // Test vertices
            test(cubeStructure.getVertex(0).point.x == -0.5);
            test(cubeStructure.getVertex(0).point.y == -0.5);
            test(cubeStructure.getVertex(0).point.z == 0.5);
            test(cubeStructure.getHalfEdge(cubeStructure.getVertex(0).edgeIndex).vertexIndex == 0);

            test(cubeStructure.getVertex(1).point.x == 0.5);
            test(cubeStructure.getVertex(1).point.y == -0.5);
            test(cubeStructure.getVertex(1).point.z == 0.5);
            test(cubeStructure.getHalfEdge(cubeStructure.getVertex(1).edgeIndex).vertexIndex == 1);

            test(cubeStructure.getVertex(2).point.x == 0.5);
            test(cubeStructure.getVertex(2).point.y == 0.5);
            test(cubeStructure.getVertex(2).point.z == 0.5);
            test(cubeStructure.getHalfEdge(cubeStructure.getVertex(2).edgeIndex).vertexIndex == 2);

            test(cubeStructure.getVertex(3).point.x == -0.5);
            test(cubeStructure.getVertex(3).point.y == 0.5);
            test(cubeStructure.getVertex(3).point.z == 0.5);
            test(cubeStructure.getHalfEdge(cubeStructure.getVertex(3).edgeIndex).vertexIndex == 3);

            test(cubeStructure.getVertex(4).point.x == -0.5);
            test(cubeStructure.getVertex(4).point.y == -0.5);
            test(cubeStructure.getVertex(4).point.z == -0.5);
            test(cubeStructure.getHalfEdge(cubeStructure.getVertex(4).edgeIndex).vertexIndex == 4);

            test(cubeStructure.getVertex(5).point.x == 0.5);
            test(cubeStructure.getVertex(5).point.y == -0.5);
            test(cubeStructure.getVertex(5).point.z == -0.5);
            test(cubeStructure.getHalfEdge(cubeStructure.getVertex(5).edgeIndex).vertexIndex == 5);

            test(cubeStructure.getVertex(6).point.x == 0.5);
            test(cubeStructure.getVertex(6).point.y == 0.5);
            test(cubeStructure.getVertex(6).point.z == -0.5);
            test(cubeStructure.getHalfEdge(cubeStructure.getVertex(6).edgeIndex).vertexIndex == 6);

            test(cubeStructure.getVertex(7).point.x == -0.5);
            test(cubeStructure.getVertex(7).point.y == 0.5);
            test(cubeStructure.getVertex(7).point.z == -0.5);
            test(cubeStructure.getHalfEdge(cubeStructure.getVertex(7).edgeIndex).vertexIndex == 7);

            // Test faces
            for (uint f=0; f<6; f++) {
                test(cubeStructure.getHalfEdge(cubeStructure.getFace(f).edgeIndex).faceIndex == f);
            }

            // Test edges
            for (uint f=0; f<6; f++) {


                uint edgeIndex = cubeStructure.getFace(f).edgeIndex;
                const uint firstEdgeIndex = edgeIndex;

                // For each half-edge of the face
                for (uint e=0; e<4; e++) {

                    rp3d::HalfEdgeStructure::Edge edge = cubeStructure.getHalfEdge(edgeIndex);

                    test(cubeStructure.getHalfEdge(edge.twinEdgeIndex).twinEdgeIndex == edgeIndex);
                    test(edge.faceIndex == f);

                    // Go to the next edge
                    edgeIndex = edge.nextEdgeIndex;
                }

                test(firstEdgeIndex == edgeIndex);
            }
        }

        void testTetrahedron() {

            // Create the half-edge structure for a tetrahedron
            std::vector<rp3d::Vector3> vertices;
            std::vector<std::vector<uint>> faces;
            rp3d::HalfEdgeStructure tetrahedron;

            // Vertices
            vertices.push_back(rp3d::Vector3(1, -1, -1));
            vertices.push_back(rp3d::Vector3(-1, -1, -1));
            vertices.push_back(rp3d::Vector3(0, -1, 1));
            vertices.push_back(rp3d::Vector3(0, 1, 0));

            // Faces
            std::vector<uint> face0;
            face0.push_back(0); face0.push_back(1); face0.push_back(2);
            std::vector<uint> face1;
            face1.push_back(0); face1.push_back(3); face1.push_back(1);
            std::vector<uint> face2;
            face2.push_back(1); face2.push_back(3); face2.push_back(2);
            std::vector<uint> face3;
            face3.push_back(0); face3.push_back(2); face3.push_back(3);

            faces.push_back(face0);
            faces.push_back(face1);
            faces.push_back(face2);
            faces.push_back(face3);

            tetrahedron.init(vertices, faces);

            // --- Test that the half-edge structure of the tetrahedron is valid --- //

            test(tetrahedron.getNbFaces() == 4);
            test(tetrahedron.getNbVertices() == 4);
            test(tetrahedron.getNbHalfEdges() == 12);

            // Test vertices
            test(tetrahedron.getVertex(0).point.x == 1);
            test(tetrahedron.getVertex(0).point.y == -1);
            test(tetrahedron.getVertex(0).point.z == -1);
            test(tetrahedron.getHalfEdge(tetrahedron.getVertex(0).edgeIndex).vertexIndex == 0);

            test(tetrahedron.getVertex(1).point.x == -1);
            test(tetrahedron.getVertex(1).point.y == -1);
            test(tetrahedron.getVertex(1).point.z == -1);
            test(tetrahedron.getHalfEdge(tetrahedron.getVertex(1).edgeIndex).vertexIndex == 1);

            test(tetrahedron.getVertex(2).point.x == 0);
            test(tetrahedron.getVertex(2).point.y == -1);
            test(tetrahedron.getVertex(2).point.z == 1);
            test(tetrahedron.getHalfEdge(tetrahedron.getVertex(2).edgeIndex).vertexIndex == 2);

            test(tetrahedron.getVertex(3).point.x == 0);
            test(tetrahedron.getVertex(3).point.y == 1);
            test(tetrahedron.getVertex(3).point.z == 0);
            test(tetrahedron.getHalfEdge(tetrahedron.getVertex(3).edgeIndex).vertexIndex == 3);

            // Test faces
            for (uint f=0; f<4; f++) {
                test(tetrahedron.getHalfEdge(tetrahedron.getFace(f).edgeIndex).faceIndex == f);
            }

            // Test edges
            for (uint f=0; f<4; f++) {

                uint edgeIndex = tetrahedron.getFace(f).edgeIndex;
                const uint firstEdgeIndex = edgeIndex;

                // For each half-edge of the face
                for (uint e=0; e<3; e++) {

                    rp3d::HalfEdgeStructure::Edge edge = tetrahedron.getHalfEdge(edgeIndex);

                    test(tetrahedron.getHalfEdge(edge.twinEdgeIndex).twinEdgeIndex == edgeIndex);
                    test(edge.faceIndex == f);

                    // Go to the next edge
                    edgeIndex = edge.nextEdgeIndex;
                }

                test(firstEdgeIndex == edgeIndex);
            }
        }
 };

}

#endif
