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

#ifndef AABB_H
#define AABB_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>

// Class AABB
class AABB  {

    private :

        // -------------------- Attributes -------------------- //

        /// Scaling matrix (applied to a cube to obtain the correct box dimensions)
        openglframework::Matrix4 mScalingMatrix;

        /// Vertex Buffer Object for the vertices data
        static openglframework::VertexBufferObject mVBOVertices;

        /// Vertex Buffer Object for the normals data
        static openglframework::VertexBufferObject mVBONormals;

        /// Vertex Buffer Object for the indices
        static openglframework::VertexBufferObject mVBOIndices;

        /// Vertex Array Object for the vertex data
        static openglframework::VertexArrayObject mVAO;

        // -------------------- Methods -------------------- //

        /// Create a the VAO and VBOs to render to box with OpenGL
        static void createVBOAndVAO();

    public :

        // -------------------- Methods -------------------- //

        /// Initialize the data to render AABBs
        static void init();

        /// Destroy the data used to render AABBs
        static void destroy();

        /// Render the cube at the correct position and with the correct orientation
        static void render(const openglframework::Vector3& position, const openglframework::Vector3& dimension,
                    openglframework::Color color, openglframework::Shader& shader,
                    const openglframework::Matrix4& worldToCameraMatrix);
};

#endif
