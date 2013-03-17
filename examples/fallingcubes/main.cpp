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

// Libraries
#include <stdlib.h>
#include <reactphysics3d.h>
#include "Box.h"

// Prototypes
void init();
void display();
void simulate();
void clean();
void reshape(int w, int h);

// Use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constants
const double FLOOR_SIZE = 20;
const double FLOOR_THICKNESS = 0.02;

// Global variables
DynamicsWorld* dynamicsWorld;   // Dynamics world
Box* boxes[2];                  // Falling boxes
BoxShape* collisionShapeBox1;   // Collision shape of the first box
BoxShape* collisionShapeBox2;   // Collision shape of the second box
BoxShape* collisionShapeFloor;  // Collision shape of the floor
RigidBody* floorRigidBody;      // Rigid body corresponding the floor


// Simulation function
void simulate() {

    // Update the physics simulation
    dynamicsWorld->update();

    // Display the scene
    display();
}

// Main function
int main(int argc, char** argv) {

    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("ReactPhysics3D Example - Falling Cubes");

    init();

    glutIdleFunc(simulate);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMainLoop();

    // Stop the physics simulation
    dynamicsWorld->stop();

    clean();
    
    return 0;
}

// Initialization function
void init() {

    glClearColor(0.0, 0.0, 0.0, 0.0);

    // Light
    glShadeModel(GL_SMOOTH);
    GLfloat light_position[] = {5.0f, 5.0f, 5.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);

    // Gravity vector of the physics world
    Vector3 gravity(0.0, -9.81, 0.0);

    // Timestep of the simulation
    decimal timeStep = 1.0/60.0;

    // Create the dynamics world
    dynamicsWorld = new DynamicsWorld(gravity, timeStep);

    // --- Create a falling box with a size of 1m and weight of 3kg --- //

    float size = 1.0f;

    // Initial position and orientation of the box
    Vector3 positionBox1(-2.0f, 7.0f, 0.0f);
    Quaternion orientationBox1(0.3, 1.0, 0.8, 0.0);
    Transform initTransform(positionBox1, orientationBox1);

    // Create a box collision shape for the box (used for collision detection)
    collisionShapeBox1 = new BoxShape(Vector3(size/2.0f, size/2.0f, size/2.0f));

    // Compute the inertia tensor of the box using the collision shape
    Matrix3x3 inertiaTensorBox1;
    float massBox1 = 3.0f;
    collisionShapeBox1->computeLocalInertiaTensor(inertiaTensorBox1, massBox1);

    // Create the rigid body associated with the box in the dynamics world
    RigidBody* rigidBody = dynamicsWorld->createRigidBody(initTransform, massBox1,
                                                          inertiaTensorBox1, collisionShapeBox1);

    // Set the contact velocity restitution factor of the rigid body
    rigidBody->setRestitution(0.5f);

    // Create the box object (used for display)
    boxes[0] = new Box(size, rigidBody);

    // --- Create a second falling box with a size of 1.5m and weight of 4.5kg --- //

    size = 1.5;

    // Initial position and orientation of the box
    Vector3 positionBox2(2.0, 4.0, 0.0);
    Quaternion orientationBox2(1.0, 1.0, 0.5, 0.0);
    Transform initTransform2(positionBox2, orientationBox2);

    // Create a box collision shape for the box (used for collision detection)
    collisionShapeBox2 = new BoxShape(Vector3(size/2.0f, size/2.0f, size/2.0f));

    // Compute the inertia tensor using the collision shape
    Matrix3x3 inertiaTensorBox2;
    float massBox2 = 4.5f;
    collisionShapeBox2->computeLocalInertiaTensor(inertiaTensorBox2, massBox2);

    // Create the rigid body associated with the box in the dynamcis world
    RigidBody* rigidBody2 = dynamicsWorld->createRigidBody(initTransform2, massBox2,
                                                           inertiaTensorBox2, collisionShapeBox2);

    // Set the contact velocity restitution factor of the rigid body
    rigidBody2->setRestitution(0.5);

    // Create the box object (used for display)
    boxes[1] = new Box(size, rigidBody2);

    // --- Create the rigid body corresponding to the floor --- //

    // Initial position and orientation of the floor
    Vector3 positionFloor(0.0, 0.0, 0.0);
    Quaternion orientationFloor(0.0, 1.0, 0.0, 0.0);
    Transform initTransformFloor(positionFloor, orientationFloor);

    // Create a box collision shape for the floor (used for collision detection)
    collisionShapeFloor = new BoxShape(Vector3(FLOOR_SIZE, FLOOR_THICKNESS, FLOOR_SIZE));

    // Compute the inertia tensor of the floor using the collision shape
    float massFloor = 100.0f;
    rp3d::Matrix3x3 inertiaTensorFloor;
    collisionShapeFloor->computeLocalInertiaTensor(inertiaTensorFloor, massFloor);

    // Create the rigid body associated with the floor in the dynamcis world
    floorRigidBody = dynamicsWorld->createRigidBody(initTransformFloor, massFloor,
                                                    inertiaTensorFloor, collisionShapeFloor);

    // The floor is a rigid body that cannot move
    floorRigidBody->setIsMotionEnabled(false);

    // Set the contact velocity restitution factor of the floor
    floorRigidBody->setRestitution(0.5);

    // Start the dynamics simulation
    dynamicsWorld->start();
}

// Display function
void display() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Display each falling box of the scene
    for (int i=0; i<2; i++) {
        boxes[i]->draw();
    }

    // Display the plane for the floor
    glBegin(GL_POLYGON);
        glNormal3f(0.0, 1.0, 0.0);
        glVertex3f(-FLOOR_SIZE/2, 0.0, -FLOOR_SIZE/2);
        glVertex3f(-FLOOR_SIZE/2, 0.0, FLOOR_SIZE/2);
        glVertex3f(FLOOR_SIZE/2, 0.0, FLOOR_SIZE/2);
        glVertex3f(FLOOR_SIZE/2, 0.0, -FLOOR_SIZE/2);
    glEnd();

    glutSwapBuffers();
}

// Reshape function
void reshape(int w, int h) {
    float ratio = ((float)w / h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, w, h);
    gluPerspective(45, ratio,1,1000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(20.0, 4.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

// Clean the memory allocation
void clean() {

    // Destroy the rigid bodies from the dynamics world
    dynamicsWorld->destroyRigidBody(boxes[0]->getRigidBodyPointer());
    dynamicsWorld->destroyRigidBody(boxes[1]->getRigidBodyPointer());
    dynamicsWorld->destroyRigidBody(floorRigidBody);

    // Destroy the dynamics world
    delete dynamicsWorld;

    // Destroy the boxes
    delete boxes[0];
    delete boxes[1];

    // Destroy the collision shapes
    delete collisionShapeBox1;
    delete collisionShapeBox2;
    delete collisionShapeFloor;
}
