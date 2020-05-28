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

#ifndef COLLISION_SHAPES_SCENE_H
#define COLLISION_SHAPES_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "SceneDemo.h"
#include "Sphere.h"
#include "Box.h"
#include "Capsule.h"
#include "ConvexMesh.h"
#include "ConcaveMesh.h"
#include "Dumbbell.h"
#include "VisualContactPoint.h"

namespace collisionshapesscene {

// Constants
const float SCENE_RADIUS = 30.0f;
const int NB_BOXES = 10;
const int NB_SPHERES = 5;
const int NB_CAPSULES = 5;
const int NB_MESHES = 5;
const int NB_COMPOUND_SHAPES = 5;
const openglframework::Vector3 BOX_SIZE(2, 2, 2);
const float SPHERE_RADIUS = 1.5f;
const float CONE_RADIUS = 2.0f;
const float CONE_HEIGHT = 3.0f;
const float CYLINDER_RADIUS = 1.0f;
const float CYLINDER_HEIGHT = 5.0f;
const float CAPSULE_RADIUS = 1.0f;
const float CAPSULE_HEIGHT = 1.0f;
const float DUMBBELL_HEIGHT = 1.0f;
const openglframework::Vector3 FLOOR_SIZE(50, 0.5f, 50);        // Floor dimensions in meters

// Class CollisionShapesScene
class CollisionShapesScene : public SceneDemo {

    private :

        // -------------------- Attributes -------------------- //

        /// All the boxes of the scene
        std::vector<Box*> mBoxes;

        std::vector<Sphere*> mSpheres;

        std::vector<Capsule*> mCapsules;

        /// All the convex meshes of the scene
        std::vector<ConvexMesh*> mConvexMeshes;

        /// All the dumbbell of the scene
        std::vector<Dumbbell*> mDumbbells;

        /// Box for the floor
        Box* mFloor;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionShapesScene(const std::string& name, EngineSettings& settings);

        /// Destructor
        virtual ~CollisionShapesScene() override;

        /// Reset the scene
        virtual void reset() override;
};

}

#endif
