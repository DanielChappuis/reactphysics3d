/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_JOINT_COMPONENTS_H
#define REACTPHYSICS3D_JOINT_COMPONENTS_H

// Libraries
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/components/Components.h>
#include <reactphysics3d/containers/Map.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;
class Joint;
enum class JointType;

// Class JointComponents
/**
 * This class represent the component of the ECS that contains generic information about
 * all the joints.
 */
class JointComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of joint entities
        Entity* mJointEntities;

        /// Array of body entities of the first bodies of the joints
        Entity* mBody1Entities;

        /// Array of body entities of the first bodies of the joints
        Entity* mBody2Entities;

        /// Array with pointers to the joints
        Joint** mJoints;

        /// Array of type of the joints
        JointType* mTypes;

        /// Array of position correction techniques used for the joints
        JointsPositionCorrectionTechnique* mPositionCorrectionTechniques;

        /// Array of boolean values to know if the two bodies of the constraint are allowed to collide with each other
        bool* mIsCollisionEnabled;

        /// True if the joint has already been added into an island during islands creation
        bool* mIsAlreadyInIsland;

        // -------------------- Methods -------------------- //

        /// Allocate memory for a given number of components
        virtual void allocate(uint32 nbComponentsToAllocate) override;

        /// Destroy a component at a given index
        virtual void destroyComponent(uint32 index) override;

        /// Move a component from a source to a destination index in the components array
        virtual void moveComponentToIndex(uint32 srcIndex, uint32 destIndex) override;

        /// Swap two components in the array
        virtual void swapComponents(uint32 index1, uint32 index2) override;

    public:

        /// Structure for the data of a transform component
        struct JointComponent {

            const Entity body1Entity;
            const Entity body2Entity;
            Joint* joint;
            JointType jointType;
            JointsPositionCorrectionTechnique positionCorrectionTechnique;
            bool isCollisionEnabled;

            /// Constructor
            JointComponent(Entity body1Entity, Entity body2Entity, Joint* joint, JointType jointType,
                           JointsPositionCorrectionTechnique positionCorrectionTechnique, bool isCollisionEnabled)
                : body1Entity(body1Entity), body2Entity(body2Entity), joint(joint), jointType(jointType),
                  positionCorrectionTechnique(positionCorrectionTechnique), isCollisionEnabled(isCollisionEnabled)  {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        JointComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~JointComponents() override = default;

        /// Add a component
        void addComponent(Entity jointEntity, bool isSleeping, const JointComponent& component);

        /// Return the entity of the first body of a joint
        Entity getBody1Entity(Entity jointEntity) const;

        /// Return the entity of the second body of a joint
        Entity getBody2Entity(Entity jointEntity) const;

        /// Return a pointer to the joint
        Joint* getJoint(Entity jointEntity) const;

        /// Return the type of a joint
        JointType getType(Entity jointEntity) const;

        /// Return the position correction technique of a joint
        JointsPositionCorrectionTechnique getPositionCorrectionTechnique(Entity jointEntity) const;

        /// Set the position correction technique of a joint
        void setPositionCorrectionTechnique(Entity jointEntity, JointsPositionCorrectionTechnique positionCorrectionTechnique);

        /// Return true if the collision is enabled between the two bodies of a joint
        bool getIsCollisionEnabled(Entity jointEntity) const;

        /// Set whether the collision is enabled between the two bodies of a joint
        void setIsCollisionEnabled(Entity jointEntity, bool isCollisionEnabled);

        /// Return true if the joint has already been added into an island during island creation
        bool getIsAlreadyInIsland(Entity jointEntity) const;

        /// Set to true if the joint has already been added into an island during island creation
        void setIsAlreadyInIsland(Entity jointEntity, bool isAlreadyInIsland);

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
        friend class ConstraintSolverSystem;
        friend class PhysicsWorld;
        friend class SolveBallAndSocketJointSystem;
        friend class SolveFixedJointSystem;
        friend class SolveHingeJointSystem;
        friend class SolveSliderJointSystem;

};

// Return the entity of the first body of a joint
RP3D_FORCE_INLINE Entity JointComponents::getBody1Entity(Entity jointEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBody1Entities[mMapEntityToComponentIndex[jointEntity]];
}

// Return the entity of the second body of a joint
RP3D_FORCE_INLINE Entity JointComponents::getBody2Entity(Entity jointEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBody2Entities[mMapEntityToComponentIndex[jointEntity]];
}

// Return a pointer to the joint
RP3D_FORCE_INLINE Joint* JointComponents::getJoint(Entity jointEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mJoints[mMapEntityToComponentIndex[jointEntity]];
}

// Return the type of a joint
RP3D_FORCE_INLINE JointType JointComponents::getType(Entity jointEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mTypes[mMapEntityToComponentIndex[jointEntity]];
}

// Return the position correction technique of a joint
RP3D_FORCE_INLINE JointsPositionCorrectionTechnique JointComponents::getPositionCorrectionTechnique(Entity jointEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mPositionCorrectionTechniques[mMapEntityToComponentIndex[jointEntity]];
}

// Set the position correction technique of a joint
RP3D_FORCE_INLINE void JointComponents::setPositionCorrectionTechnique(Entity jointEntity, JointsPositionCorrectionTechnique positionCorrectionTechnique) {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mPositionCorrectionTechniques[mMapEntityToComponentIndex[jointEntity]] = positionCorrectionTechnique;
}

// Return true if the collision is enabled between the two bodies of a joint
RP3D_FORCE_INLINE bool JointComponents::getIsCollisionEnabled(Entity jointEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mIsCollisionEnabled[mMapEntityToComponentIndex[jointEntity]];
}

// Set whether the collision is enabled between the two bodies of a joint
RP3D_FORCE_INLINE void JointComponents::setIsCollisionEnabled(Entity jointEntity, bool isCollisionEnabled) {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mIsCollisionEnabled[mMapEntityToComponentIndex[jointEntity]] = isCollisionEnabled;
}

// Return true if the joint has already been added into an island during island creation
RP3D_FORCE_INLINE bool JointComponents::getIsAlreadyInIsland(Entity jointEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mIsAlreadyInIsland[mMapEntityToComponentIndex[jointEntity]];
}

// Set to true if the joint has already been added into an island during island creation
RP3D_FORCE_INLINE void JointComponents::setIsAlreadyInIsland(Entity jointEntity, bool isAlreadyInIsland) {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mIsAlreadyInIsland[mMapEntityToComponentIndex[jointEntity]] = isAlreadyInIsland;
}

}

#endif
