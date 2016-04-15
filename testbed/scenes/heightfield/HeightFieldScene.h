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

#ifndef HEIGHT_FIELD_SCENE_H
#define HEIGHT_FIELD_SCENE_H

// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"
#include "Box.h"
#include "SceneDemo.h"
#include "HeightField.h"

namespace heightfieldscene {

// Constants
const float SCENE_RADIUS = 50.0f;

// Class HeightFieldScene
class HeightFieldScene : public SceneDemo {

    static const int NB_BOXES = 10;

    protected :

        // -------------------- Attributes -------------------- //

        Box* mBoxes[NB_BOXES];

        /// Height field
        HeightField* mHeightField;

        /// Dynamics world used for the physics simulation
        rp3d::DynamicsWorld* mDynamicsWorld;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        HeightFieldScene(const std::string& name);

        /// Destructor
        virtual ~HeightFieldScene();

        /// Update the physics world (take a simulation step)
        /// Can be called several times per frame
        virtual void updatePhysics();

        /// Update the scene (take a simulation step)
        virtual void update();

        /// Render the scene in a single pass
        virtual void renderSinglePass(openglframework::Shader& shader,
                                      const openglframework::Matrix4& worldToCameraMatrix);

        /// Reset the scene
        virtual void reset();

        /// Return all the contact points of the scene
        virtual std::vector<ContactPoint> getContactPoints() const;
};

// Return all the contact points of the scene
inline std::vector<ContactPoint> HeightFieldScene::getContactPoints() const {
    return computeContactPointsOfWorld(mDynamicsWorld);
}

}

#endif
