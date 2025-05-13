/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2025 Grzegorz Szczodrzec
* based on examples provided by library
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

#ifndef TEST_DYNAMICSYSTEM_H
#define TEST_DYNAMICSYSTEM_H
// Libraries
#include <reactphysics3d/reactphysics3d.h>
#include "Test.h"
/// Reactphysics3D namespace
namespace reactphysics3d {

/**
 * Unit test for the DynamicSystem class.
 */

class TestDynamicSystem : public Test {
private:
  PhysicsCommon mPhysicsCommon;
  PhysicsWorld* mWorld = nullptr;
  RigidBody* mRigidBody1 = nullptr;
  RigidBody* mRigidBody2 = nullptr;
    
public:
  // ---------- Methods ---------- //
  /// Constructor
  TestDynamicSystem(const std::string& name);
       
  /// Destructor 
  virtual ~TestDynamicSystem();
        
  /// Run the tests
  virtual void run() override { testMotionIntegration(); }
        
  // @brief integerates motion of Rigid Body and checks if angular momentum 
  // remains constant.
  //
  // In absence of gravity and without colisions there is no source of torque.
  // Without torque angular momentum have to be constant.
  void testMotionIntegration();
        
  Vector3 getAngularMomentum(RigidBody* pBody);
};

inline
TestDynamicSystem::TestDynamicSystem(const std::string& name) : Test(name) {
            
  mWorld = mPhysicsCommon.createPhysicsWorld();
  mWorld->setIsGravityEnabled(false); 
    
/*
  Trying to make simulation of rotating stick to be close to reality. 
  wooden stick 1m in length 2x2cm of cross-section
  rho = 600 kg/m^3;
  m=rho*a*b*c;
  I = m(a^2+b^2)/12;
 */
  const decimal rho = 600; // kg/m^3
  const decimal edge = 0.02; // 2 cm
  const decimal stickLen = 1; // 1 m
  // Create a rigid body in the world CS
  Vector3 position(0, 2.0, 0);
  Quaternion orientation = Quaternion::identity();
  Transform transform(position, orientation);

  mRigidBody1 = mWorld->createRigidBody(transform);

  decimal mass = rho * edge * edge * stickLen;
  mRigidBody1->setMass(mass);
  { // inertia
    decimal weak = mass * 2 * edge*edge /12;
    decimal strong =  mass * (edge*edge + stickLen*stickLen)/12;
    // local X is along stick
    mRigidBody1->setLocalInertiaTensor( Vector3(weak, strong, strong) );
  }
  // kinematics
  {
    decimal omegaW = 1.0 / (stickLen/2);
    mRigidBody1->setAngularVelocity(Vector3(0.1*omegaW, omegaW , 0));
  }
}

inline 
void TestDynamicSystem::testMotionIntegration(){

    Vector3 initialAngularMomentum =  getAngularMomentum(mRigidBody1);
    const decimal timeStep = 1.e-2 / 60.0f;
	// Step the simulation a few steps
	for (int i=0; i < 400; i++) {
	  mWorld->update(timeStep);
    }
    Vector3 finalAngularMomentum =  getAngularMomentum(mRigidBody1);
    
    rp3d_test(Vector3::approxEqual(initialAngularMomentum, finalAngularMomentum)); 
}

inline 
Vector3 TestDynamicSystem::getAngularMomentum(RigidBody* pBody){
  const Vector3 & Inertia = pBody->getLocalInertiaTensor();
  const Vector3 & omegaW = pBody ->getAngularVelocity();
  const Matrix3x3 & B2W = pBody->getTransform().getOrientation().getMatrix();

  Vector3 omegaB = B2W.getTranspose() * omegaW;

  Vector3 angMomentum = B2W * ( omegaB * Inertia ); // elwise product
  return angMomentum;
}
        
inline TestDynamicSystem::~TestDynamicSystem(){
  if(mRigidBody1)
    mWorld->destroyRigidBody(mRigidBody1);
  if(mRigidBody2)
    mWorld->destroyRigidBody(mRigidBody2);
}


} // namespace
#endif // TEST_DYNAMICSYSTEM_H
