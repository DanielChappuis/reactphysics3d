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

#ifndef TRIANGLE_MESH_SCENE_H
#define TRIANGLE_MESH_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Box.h"
#include "SceneDemo.h"
#include "ConcaveMesh.h"
#include "Box.h"
#include "Capsule.h"
#include "Dumbbell.h"
#include "Sphere.h"
#include "ConvexMesh.h"

namespace trianglemeshscene {

// Constants
const float SCENE_RADIUS = 70.0f;                           // Radius of the scene in meters
static const int NB_BOXES = 50;
static const int NB_SPHERES = 40;
static const int NB_CAPSULES = 20;
static const int NB_MESHES = 15;
static const int NB_COMPOUND_SHAPES = 3;
const openglframework::Vector3 BOX_SIZE(2, 2, 2);
const float SPHERE_RADIUS = 1.5f;
const float CONE_RADIUS = 2.0f;
const float CONE_HEIGHT = 3.0f;
const float CYLINDER_RADIUS = 1.0f;
const float CYLINDER_HEIGHT = 5.0f;
const float CAPSULE_RADIUS = 1.0f;
const float CAPSULE_HEIGHT = 1.0f;
const float DUMBBELL_HEIGHT = 1.0f;

// Class TriangleMeshScene
class ConcaveMeshScene : public SceneDemo {

    protected :

        // -------------------- Attributes -------------------- //

        std::vector<Box*> mBoxes;

        std::vector<Sphere*> mSpheres;

        std::vector<Capsule*> mCapsules;

        /// All the convex meshes of the scene
        std::vector<ConvexMesh*> mConvexMeshes;

        /// All the dumbbell of the scene
        std::vector<Dumbbell*> mDumbbells;

        /// Concave triangles mesh
        ConcaveMesh* mConcaveMesh;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ConcaveMeshScene(const std::string& name, EngineSettings& settings);

        /// Destructor
        virtual ~ConcaveMeshScene() override;

        /// Reset the scene
        virtual void reset() override;
};

}

#endif
