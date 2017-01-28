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
#include "reactphysics3d.h"
#include "SceneDemo.h"
#include "Sphere.h"
#include "Box.h"
#include "Cone.h"
#include "Cylinder.h"
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
const float CAPSULE_RADIUS = 3.0f;
const float CAPSULE_HEIGHT = 5.0f;
const float DUMBBELL_HEIGHT = 5.0f;
const int NB_RAYS = 100;
const float RAY_LENGTH = 30.0f;
const int NB_BODIES = 9;

// Raycast manager
class ContactManager : public rp3d::CollisionCallback {

    private:

        /// All the visual contact points
        std::vector<ContactPoint> mContactPoints;

        /// All the normals at contact points
        std::vector<Line*> mNormals;

        /// Contact point mesh folder path
        std::string mMeshFolderPath;

   public:

        ContactManager(openglframework::Shader& shader,
                       const std::string& meshFolderPath)
            : mMeshFolderPath(meshFolderPath) {

        }

        /// This method will be called for each reported contact point
        virtual void notifyContact(const CollisionCallbackInfo& collisionCallbackInfo) override {

            rp3d::Vector3 point1 = collisionCallbackInfo.contactPoint.localPoint1;
            point1 = collisionCallbackInfo.proxyShape1->getLocalToWorldTransform() * point1;
            openglframework::Vector3 position1(point1.x, point1.y, point1.z);
            mContactPoints.push_back(ContactPoint(position1));

            rp3d::Vector3 point2 = collisionCallbackInfo.contactPoint.localPoint2;
            point2 = collisionCallbackInfo.proxyShape2->getLocalToWorldTransform() * point2;
            openglframework::Vector3 position2(point2.x, point2.y, point2.z);
            mContactPoints.push_back(ContactPoint(position2));

            // Create a line to display the normal at hit point
            rp3d::Vector3 n = collisionCallbackInfo.contactPoint.normal;
            openglframework::Vector3 normal(n.x, n.y, n.z);
            Line* normalLine = new Line(position1, position1 + normal);
            mNormals.push_back(normalLine);
        }

        void resetPoints() {

            mContactPoints.clear();

            // Destroy all the normals
            for (std::vector<Line*>::iterator it = mNormals.begin();
                 it != mNormals.end(); ++it) {
                delete (*it);
            }
            mNormals.clear();
        }

        std::vector<ContactPoint> getContactPoints() const {
            return mContactPoints;
        }
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
        //Cone* mCone;
        //Cylinder* mCylinder;
        //Capsule* mCapsule;
        //ConvexMesh* mConvexMesh;
        //Dumbbell* mDumbbell;
        //ConcaveMesh* mConcaveMesh;
        //HeightField* mHeightField;

        std::vector<openglframework::Object3D*> mAllShapesObjects;
        std::vector<PhysicsObject*> mAllShapesPhysicsObjects;

        int mSelectedShapeIndex;

        /// Collision world used for the physics simulation
        rp3d::CollisionWorld* mCollisionWorld;

        /// All the points to render the lines
        std::vector<openglframework::Vector3> mLinePoints;

        /// Vertex Buffer Object for the vertices data
        openglframework::VertexBufferObject mVBOVertices;

        /// Vertex Array Object for the vertex data
        openglframework::VertexArrayObject mVAO;

        /// Create the Vertex Buffer Objects used to render with OpenGL.
        void createVBOAndVAO(openglframework::Shader& shader);

        /// Select the next shape
        void selectNextShape();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionDetectionScene(const std::string& name);

        /// Destructor
        virtual ~CollisionDetectionScene() override;

        /// Update the physics world (take a simulation step)
        /// Can be called several times per frame
        virtual void updatePhysics() override;

        /// Take a step for the simulation
        virtual void update() override;

        /// Render the scene in a single pass
        virtual void renderSinglePass(openglframework::Shader& shader,
                                      const openglframework::Matrix4& worldToCameraMatrix) override;

        /// Reset the scene
        virtual void reset() override;

        /// Display or not the surface normals at hit points
        void showHideNormals();

        /// Called when a keyboard event occurs
        virtual bool keyboardEvent(int key, int scancode, int action, int mods) override;

        /// Enabled/Disable the shadow mapping
        virtual void setIsShadowMappingEnabled(bool isShadowMappingEnabled) override;

        /// Display/Hide the contact points
        virtual void setIsContactPointsDisplayed(bool display) override;

        /// Return all the contact points of the scene
        virtual std::vector<ContactPoint> getContactPoints() const override;
};

// Display or not the surface normals at hit points
inline void CollisionDetectionScene::showHideNormals() {
    mAreNormalsDisplayed = !mAreNormalsDisplayed;
}

// Enabled/Disable the shadow mapping
inline void CollisionDetectionScene::setIsShadowMappingEnabled(bool isShadowMappingEnabled) {
    SceneDemo::setIsShadowMappingEnabled(false);
}

// Display/Hide the contact points
inline void CollisionDetectionScene::setIsContactPointsDisplayed(bool display) {
    SceneDemo::setIsContactPointsDisplayed(true);
}

// Return all the contact points of the scene
inline std::vector<ContactPoint> CollisionDetectionScene::getContactPoints() const {
    return mContactManager.getContactPoints();
}

}

#endif
