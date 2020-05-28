/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2013 Daniel Chappuis                                            *
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

// Libraries
#include "MeshReaderWriter.h"
#include <fstream>
#include <sstream>
#include <locale>
#include <cctype>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <tuple>

using namespace openglframework;
using namespace std;

// Constructor
MeshReaderWriter::MeshReaderWriter() {

}

// Load a mesh from a file and returns true if the mesh has been sucessfully loaded
void MeshReaderWriter::loadMeshFromFile(const std::string& filename, Mesh& meshToCreate) {

    // Get the extension of the file
    uint startPosExtension = filename.find_last_of(".");
    string extension = filename.substr(startPosExtension+1);

    // Load the file using the correct method
    if (extension == "obj") {
        loadOBJFile(filename, meshToCreate);
    }
    else {

        // Display an error message and throw an exception
        string errorMessage("Error : the MeshReaderWriter class cannot load a file with the extension .");
        errorMessage += extension;
        std::cerr << errorMessage << std::endl;
        throw std::invalid_argument(errorMessage.c_str());
    }
}

// Write a mesh to a file
void MeshReaderWriter::writeMeshToFile(const std::string& filename, const Mesh& meshToWrite) {

    // Get the extension of the file
    uint startPosExtension = filename.find_last_of(".");
    string extension = filename.substr(startPosExtension+1);

    // Load the file using the correct method
    if (extension == "obj") {
        writeOBJFile(filename, meshToWrite);
    }
    else {

        // Display an error message and throw an exception
        string errorMessage("Error : the MeshReaderWriter class cannot store a mesh file with the extension .");
        errorMessage += extension;
        std::cerr << errorMessage << std::endl;
        throw std::invalid_argument(errorMessage.c_str());
    }
}

// Load an OBJ file with a triangular or quad mesh
void MeshReaderWriter::loadOBJFile(const string &filename, Mesh& meshToCreate) {

    // Open the file
    std::ifstream meshFile(filename.c_str());

    // If we cannot open the file
    if(!meshFile.is_open()) {

        // Throw an exception and display an error message
        string errorMessage("Error : Cannot open the file " + filename);
        std::cerr << errorMessage << std::endl;
        throw runtime_error(errorMessage);
    }

    std::string buffer;
    string line, tmp;
    int id1, id2, id3, id4;
    int nId1, nId2, nId3, nId4;
    int tId1, tId2, tId3, tId4;
    float v1, v2, v3;
    size_t found1, found2;
    std::vector<bool> isQuad;
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
    std::vector<uint> verticesIndices;
    std::vector<uint> normalsIndices;
    std::vector<uint> uvsIndices;


    // ---------- Collect the data from the file ---------- //

    // For each line of the file
    while(std::getline(meshFile, buffer)) {

        std::istringstream lineStream(buffer);
        std::string word;
        lineStream >> word;
        std::transform(word.begin(), word.end(), word.begin(), ::tolower);
        if(word == "usemtl") {  // Material definition

            // Loading of MTL file is not implemented

        }
        else if(word == "v") {  // Vertex position
            sscanf(buffer.c_str(), "%*s %f %f %f", &v1, &v2, &v3);
            vertices.push_back(Vector3(v1, v2, v3));
        }
        else if(word == "vt") { // Vertex texture coordinate
            sscanf(buffer.c_str(), "%*s %f %f", &v1, &v2);
            uvs.push_back(Vector2(v1,v2));
        }
        else if(word == "vn") { // Vertex normal
            sscanf(buffer.c_str(), "%*s %f %f %f", &v1, &v2, &v3);
            normals.push_back(Vector3(v1 ,v2, v3));
        }
        else if (word == "f") { // Face
            line = buffer;
            found1 = (int)line.find("/");
            bool isFaceQuad = false;
            found2 = (int)line.substr(found1+1).find("/");

            // If the face definition is of the form "f v1 v2 v3 v4"
            if(found1 == string::npos) {
                int nbVertices = sscanf(buffer.c_str(), "%*s %d %d %d %d", &id1, &id2, &id3, &id4);
                if (nbVertices == 4) isFaceQuad = true;
            }
            // If the face definition is of the form "f v1// v2// v3// v4//"
            else if (found2 == 0 && (int)line.substr(found1+found2+1).find(" ") == 0) {
                int nbVertices = sscanf(buffer.c_str(), "%*s %d// %d// %d// %d//", &id1, &id2, &id3, &id4);
                if (nbVertices == 4) isFaceQuad = true;
            }
            else {  // If the face definition contains vertices and (texture coordinates or normals)

				tId1 = -1;
				tId2 = -1;
				tId3 = -1;
				tId4 = -1;

				nId1 = -1;
				nId2 = -1;
				nId3 = -1;
				nId4 = -1;

                //get the part of the string until the second index
                tmp = line.substr(found1+1);
                found2 = (int)tmp.find(" ");
                tmp = tmp.substr(0,found2);
                found2 = (int)tmp.find("/");

                // If the face definition is of the form "f vert1/textcoord1 vert2/textcoord2 ..."
                if(found2 == string::npos) {
                    int n = sscanf(buffer.c_str(), "%*s %d/%d %d/%d %d/%d %d/%d", &id1, &tId1, &id2, &tId2, &id3, &tId3, &id4, &tId4);
                    if (n == 8) isFaceQuad = true;

                    uvsIndices.push_back(tId1-1);
                    uvsIndices.push_back(tId2-1);
                    uvsIndices.push_back(tId3-1);
                    if (isFaceQuad) uvsIndices.push_back(tId4-1);
                }
                else {
                    tmp = line.substr(found1+1);
                    found2 = (int)tmp.find("/");
					if (found2 > 1000) {
						int test = 2;
					}

                    // If the face definition is of the form "f vert1//normal1 vert2//normal2 ..."
                    if(found2 == 0) {
                        int n = sscanf(buffer.c_str(), "%*s %d//%d %d//%d %d//%d %d//%d", &id1, &nId1, &id2, &nId2, &id3, &nId3, &id4, &nId4);
                        if (n == 8) isFaceQuad = true;
                    }
                    // If the face definition is of the form "f vert1/textcoord1/normal1 ..."
                    else {
                        int n = sscanf(buffer.c_str(), "%*s %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d", &id1, &tId1, &nId1, &id2, &tId2, &nId2, &id3, &tId3, &nId3, &id4, &tId4, &nId4);
                        if (n == 12) isFaceQuad = true;
                        uvsIndices.push_back(tId1-1);
                        uvsIndices.push_back(tId2-1);
                        uvsIndices.push_back(tId3-1);
                        if (isFaceQuad) uvsIndices.push_back(tId4-1);
                    }
                    normalsIndices.push_back(nId1-1);
                    normalsIndices.push_back(nId2-1);
                    normalsIndices.push_back(nId3-1);
                    if (isFaceQuad) normalsIndices.push_back(nId4-1);
                }
            }
            verticesIndices.push_back(id1-1);
            verticesIndices.push_back(id2-1);
            verticesIndices.push_back(id3-1);
            if (isFaceQuad) verticesIndices.push_back((id4-1));
            isQuad.push_back(isFaceQuad);
        }
    }

    assert(!verticesIndices.empty());
    assert(normalsIndices.empty() || normalsIndices.size() == verticesIndices.size());
    assert(uvsIndices.empty() || uvsIndices.size() == verticesIndices.size());
    meshFile.close();

    // ---------- Merge the data that we have collected from the file ---------- //

    // Destroy the current mesh
    meshToCreate.destroy();

	// This is used to create duplicate vertices if a vertex with index "i" from a face does not
	// have same texture coordinates or normals as a previous vertex with index "i".
	unordered_map<tuple<int, int, int>, uint> mapVertNormTexToVertexIndex;

    // Mesh data
    vector<std::vector<uint> > meshIndices;
    vector<Vector3> meshVertices;
    vector<Vector3> meshNormals;
    //if (!normals.empty()) meshNormals = vector<Vector3>(vertices.size(), Vector3(0, 0, 0));
    vector<Vector2> meshUVs;
    //if (!uvs.empty()) meshUVs = vector<Vector2>(vertices.size(), Vector2(0, 0));

    // We cannot load mesh with several parts for the moment
    uint meshPart = 0;

	const bool hasNormals = !normalsIndices.empty() && !normals.empty();
	const bool hasUvs = !uvsIndices.empty() && !uvs.empty();

    // Fill in the vertex indices
    // We also triangulate each quad face
    meshIndices.push_back(std::vector<uint>());
    for(size_t i = 0, j = 0; i < verticesIndices.size(); j++) {

		// 3 if the current vertices form a triangle and 4 if they form a quad
		const int nbVertices = isQuad[j] ? 4 : 3;

		int newVerticesIndices[4] = { -1, -1, -1, -1 };

		// For each vertex, we check if there is already a vertex with same UV and normals. 
		for (int v = 0; v < nbVertices; v++) {

			int normalIndex = hasNormals ? normalsIndices[i + v] : -1;
			int uvIndex = hasUvs ? uvsIndices[i + v] : -1;

			// If the vertex with same UV and normal doesn't exist yet in the map
			tuple<int, int, int> key = std::make_tuple(verticesIndices[i+v], normalIndex, uvIndex);
			auto itFound = mapVertNormTexToVertexIndex.find(key);
			if (itFound == mapVertNormTexToVertexIndex.end()) {

				// Create a new vertex 
				newVerticesIndices[v]= meshVertices.size();
				meshVertices.push_back(vertices[verticesIndices[i+v]]);
				if (hasNormals) {
					meshNormals.push_back(normals[normalsIndices[i+v]]);
				}
				if (hasUvs) {
					meshUVs.push_back(uvs[uvsIndices[i+v]]);
				}

				mapVertNormTexToVertexIndex.insert(std::make_pair(key, newVerticesIndices[v]));
			}
			else {
				// Get the vertex index to use
				newVerticesIndices[v] = itFound->second;
			}
		}

        // Get the current vertex IDs
        uint i1 = newVerticesIndices[0];
        uint i2 = newVerticesIndices[1];
        uint i3 = newVerticesIndices[2];
        uint i4 = newVerticesIndices[3];

		/*
        // Add the vertex normal
        if (hasNormals) {
            meshNormals[i1] = normals[normalsIndices[i]];
            meshNormals[i2] = normals[normalsIndices[i+1]];
            meshNormals[i3] = normals[normalsIndices[i+2]];
        }

        // Add the vertex UV texture coordinates
        if (hasUvs) {
            meshUVs[i1] = uvs[uvsIndices[i]];
            meshUVs[i2] = uvs[uvsIndices[i+1]];
            meshUVs[i3] = uvs[uvsIndices[i+2]];
        }
		*/

        // If the current vertex not in a quad (it is part of a triangle)
        if (!isQuad[j]) {

            // Add the vertex indices
            meshIndices[meshPart].push_back(i1);
            meshIndices[meshPart].push_back(i2);
            meshIndices[meshPart].push_back(i3);

            i+=3;
        }
        else {  // If the current vertex is in a quad

            Vector3 v1 = meshVertices[i1];
            Vector3 v2 = meshVertices[i2];
            Vector3 v3 = meshVertices[i3];
            Vector3 v4 = meshVertices[i4];

            Vector3 v13 = v3-v1;
            Vector3 v12 = v2-v1;
            Vector3 v14 = v4-v1;

            float a1 = v13.dot(v12);
            float a2 = v13.dot(v14);
            if((a1 >= 0 && a2 <= 0) || (a1 <= 0 && a2 >= 0)) {
                meshIndices[meshPart].push_back(i1);
                meshIndices[meshPart].push_back(i2);
                meshIndices[meshPart].push_back(i3);
                meshIndices[meshPart].push_back(i1);
                meshIndices[meshPart].push_back(i3);
                meshIndices[meshPart].push_back(i4);
            }
            else {
                meshIndices[meshPart].push_back(i1);
                meshIndices[meshPart].push_back(i2);
                meshIndices[meshPart].push_back(i4);
                meshIndices[meshPart].push_back(i2);
                meshIndices[meshPart].push_back(i3);
                meshIndices[meshPart].push_back(i4);
            }

			/*
            // Add the vertex normal
            if (!normalsIndices.empty() && !normals.empty()) {
                meshNormals[i4] = normals[normalsIndices[i]];
            }

            // Add the vertex UV texture coordinates
            if (!uvsIndices.empty() && !uvs.empty()) {
                meshUVs[i4] = uvs[uvsIndices[i]];
            }
			*/

            i+=4;
        }
    }

    assert(meshNormals.empty() || meshNormals.size() == meshVertices.size());
    assert(meshUVs.empty() || meshUVs.size() == meshVertices.size());

    // Set the data to the mesh
    meshToCreate.setIndices(meshIndices);
    meshToCreate.setVertices(meshVertices);
    meshToCreate.setNormals(meshNormals);
    meshToCreate.setUVs(meshUVs);
}

// Store a mesh into a OBJ file
void MeshReaderWriter::writeOBJFile(const std::string& filename, const Mesh& meshToWrite) {
    std::ofstream file(filename.c_str());

    // Geth the mesh data
    const std::vector<Vector3>& vertices = meshToWrite.getVertices();
    const std::vector<Vector3>& normals = meshToWrite.getNormals();
    const std::vector<Vector2>& uvs = meshToWrite.getUVs();

    // If we can open the file
    if (file.is_open()) {

        assert(meshToWrite.getNbVertices() == vertices.size());

        // Write the vertices
        for (uint v=0; v<vertices.size(); v++) {

            file << "v " << vertices[v].x << " " << vertices[v].y << " " << vertices[v].z <<
                    std::endl;
        }

        // Write the normals
        if (meshToWrite.hasNormals()) {
            file << std::endl;

            assert(meshToWrite.getNbVertices() == normals.size());

            for (uint v=0; v<normals.size(); v++) {

                file << "vn " << normals[v].x << " " << normals[v].y << " " << normals[v].z <<
                        std::endl;
            }
        }

        // Write the UVs texture coordinates
        if (meshToWrite.hasUVTextureCoordinates()) {
            file << std::endl;

            assert(meshToWrite.getNbVertices() == uvs.size());

            for (uint v=0; v<uvs.size(); v++) {

                file << "vt " << uvs[v].x << " " << uvs[v].y << std::endl;
            }
        }

        // Write the faces
        file << std::endl;
        for (uint p=0; p<meshToWrite.getNbParts(); p++) {

            // Get the indices of the part
            const std::vector<uint>& indices = meshToWrite.getIndices(p);

            // For each index of the part
            for (uint i=0; i<indices.size(); i+=3) {

                if (meshToWrite.hasNormals() && meshToWrite.hasUVTextureCoordinates()) {
                    file << "f " <<indices[i]+1 << "/" << indices[i]+1 << "/" << indices[i]+1 <<
                            " " << indices[i+1]+1 << "/" << indices[i+1]+1 << "/" << indices[i+1]+1 <<
                            " " << indices[i+2]+1 << "/" << indices[i+2]+1 << "/" << indices[i+2]+1 <<
                            std::endl;
                }
                else if (meshToWrite.hasNormals() || meshToWrite.hasUVTextureCoordinates()) {
                    file << "f " <<indices[i]+1 << "/" << indices[i]+1 <<
                            " " << indices[i+1]+1 << "/" << indices[i+1]+1 <<
                            " " << indices[i+2]+1 << "/" << indices[i+2]+1 << std::endl;
                }
                else {
                    file << "f " << indices[i]+1 << " " << indices[i+1]+1 << " " << indices[i+2]+1 <<
                            std::endl;
                }
            }
        }
    }
    else {
        std::cerr << "Error : Cannot open the file " << filename << std::endl;
        exit(1);
    }
}
