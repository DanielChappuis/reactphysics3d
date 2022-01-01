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

#ifndef COLLISION_DETECTION_SCENE_H
#define COLLISION_DETECTION_SCENE_H

// Libraries
#include <cmath>
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "SceneDemo.h"
#include "Sphere.h"
#include "Box.h"
#include "Capsule.h"
#include "Line.h"
#include "ConvexMesh.h"
#include "ConcaveMesh.h"
#include "HeightField.h"
#include "Dumbbell.h"
#include "VisualContactPoint.h"

namespace collisiondetectionscene {

// Constants
const float SCENE_RADIUS = 30.0f;
const openglframework::Vector3 BOX_SIZE(4, 2, 1);
const float SPHERE_RADIUS = 3.0f;
const float CONE_RADIUS = 3.0f;
const float CONE_HEIGHT = 5.0f;
const float CYLINDER_RADIUS = 3.0f;
const float CYLINDER_HEIGHT = 5.0f;
const float CAPSULE_RADIUS = 1.0f;
const float CAPSULE_HEIGHT = 1.0f;
const float DUMBBELL_HEIGHT = 5.0f;
const int NB_RAYS = 100;
const float RAY_LENGTH = 30.0f;
const int NB_BODIES = 9;

// Contact manager
class ContactManager : public rp3d::CollisionCallback {

    private:

        /// Contact point mesh folder path
        std::string mMeshFolderPath;

        /// Reference to the list of all the visual contact points
        std::vector<SceneContactPoint>& mContactPoints;

   public:

        ContactManager(openglframework::Shader& /*shader*/, const std::string& meshFolderPath,
                       std::vector<SceneContactPoint>& contactPoints)
            : mMeshFolderPath(meshFolderPath), mContactPoints(contactPoints) {

        }

        /// This method is called when some contacts occur
        virtual void onContact(const CallbackData& callbackData) override;
};

// Class CollisionDetectionScene
class CollisionDetectionScene : public SceneDemo {

    private :

        // -------------------- Attributes -------------------- //

        /// Contact point mesh folder path
        std::string mMeshFolderPath;

        /// Contact manager
        ContactManager mContactManager;

        bool mAreNormalsDisplayed;

        /// All objects on the scene
        //Box* mBox;
        Sphere* mSphere1;
        Sphere* mSphere2;
		Capsule* mCapsule1;
		Capsule* mCapsule2;
		Box* mBox1;
		Box* mBox2;
        ConvexMesh* mConvexMesh;
        //Dumbbell* mDumbbell;
        ConcaveMesh* mConcaveMesh;
        HeightField* mHeightField;

        std::vector<PhysicsObject*> mAllShapes;

        unsigned int mSelectedShapeIndex;

        /// Select the next shape
        void selectNextShape();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionDetectionScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~CollisionDetectionScene() override;

        /// Take a step for the simulation
        virtual void update() override;

        /// Reset the scene
        virtual void reset() override;

        /// Display or not the surface normals at hit points
        void showHideNormals();

        /// Called when a keyboard event occurs
        virtual bool keyboardEvent(int key, int scancode, int action, int mods) override;

        /// Enabled/Disable the shadow mapping
        virtual void setIsShadowMappingEnabled(bool isShadowMappingEnabled) override;

        /// Display/Hide the contact points
        virtual void setAreContactPointsDisplayed(bool display) override;
};

// Display or not the surface normals at hit points
inline void CollisionDetectionScene::showHideNormals() {
    mAreNormalsDisplayed = !mAreNormalsDisplayed;
}

// Enabled/Disable the shadow mapping
inline void CollisionDetectionScene::setIsShadowMappingEnabled(bool /*isShadowMappingEnabled*/) {
    SceneDemo::setIsShadowMappingEnabled(false);
}

// Display/Hide the contact points
inline void CollisionDetectionScene::setAreContactPointsDisplayed(bool /*display*/) {
    SceneDemo::setAreContactPointsDisplayed(true);
}

}

#endif
