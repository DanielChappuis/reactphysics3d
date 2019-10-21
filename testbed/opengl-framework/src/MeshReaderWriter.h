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

#ifndef MESH_LOADER_H
#define MESH_LOADER_H

// Libraries
#include <string>
#include <stdexcept>
#include "Mesh.h"

namespace openglframework {

// Class MeshReaderWriter
// This class is used to read or write any mesh file in order to
// create the corresponding Mesh object. Currently, this class
// is able to read meshes of the current formats : .obj
class MeshReaderWriter {

    private :

        // -------------------- Methods -------------------- //

        // Constructor (private because we do not want instances of this class)
        MeshReaderWriter();

        // Load an OBJ file with a triangular or quad mesh
        static void loadOBJFile(const std::string& filename, Mesh& meshToCreate);

        // Store a mesh into a OBJ file
        static void writeOBJFile(const std::string& filename, const Mesh &meshToWrite);

    public :

        // -------------------- Methods -------------------- //

        // Read a mesh from a file
        static void loadMeshFromFile(const std::string& filename, Mesh& meshToCreate);

        // Write a mesh to a file
        static void writeMeshToFile(const std::string& filename, const Mesh& meshToWrite);
};

// Class VertexMergingData
// This class is used in the method to read a mesh
class VertexMergingData {

    public:
        VertexMergingData() : indexPosition(0), indexNormal(0), indexUV(0) {}
        unsigned int indexPosition;
        unsigned int indexNormal;
        unsigned int indexUV;
};

// Class VertexMergingDataComparison
// This class is used in the method to read a mesh
class VertexMergingDataComparison {

    public:
        bool operator()(const VertexMergingData& x, const VertexMergingData& y) const {
            if(x.indexPosition < y.indexPosition)
                return true;
            if(x.indexPosition == y.indexPosition && x.indexNormal < y.indexNormal)
                return true;
            if(x.indexPosition == y.indexPosition && x.indexNormal ==
                                  y.indexNormal && x.indexUV < y.indexUV)
                return true;
            return false;
    }
};

}

namespace std {
	template<>
	struct hash<std::tuple<int, int, int>>
	{
		size_t operator()(const std::tuple<int, int, int>& key) const
		{
			return std::get<0>(key) ^ std::get<1>(key) ^ std::get<2>(key);
		}
	};
}

#endif
