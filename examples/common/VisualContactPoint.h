/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef VISUAL_CONTACT_POINT_H
#define VISUAL_CONTACT_POINT_H

// Libraries
#include "openglframework.h"

const float VISUAL_CONTACT_POINT_RADIUS = 0.1f;

// Class VisualContactPoint
class VisualContactPoint : public openglframework::Object3D {

    private :

        // -------------------- Attributes -------------------- //

        /// Total number of existing contact points (static attribute)
        static int mNbTotalPoints;

        /// Sphere mesh for the visual contact point
        static openglframework::Mesh mMesh;

        /// True if the mesh has been initialized
        static bool mIsMeshInitialized;

        // -------------------- Methods -------------------- //

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        VisualContactPoint(const openglframework::Vector3& position);

        /// Destructor
        ~VisualContactPoint();

        /// Load and initialize the mesh for all the contact points
        static void createStaticData(const std::string& meshFolderPath);

        /// Destroy the mesh for the contact points
        static void destroyStaticData();

        /// Render the sphere at the correct position and with the correct orientation
        void render(openglframework::Shader& shader,
                    const openglframework::Matrix4& worldToCameraMatrix);
};

#endif
