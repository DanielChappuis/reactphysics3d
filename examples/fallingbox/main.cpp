/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

// Libraries
#include <stdlib.h>
#include <GL/freeglut.h>
#include <GL/glu.h>
#include "Box.h"
#include <reactphysics3d.h>

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
const double FLOOR_THICKNESS = 0.2;

// Global variables
PhysicsWorld* physicsWorld;     // Physics world
PhysicsEngine* physicsEngine;   // Physics engine
Box* boxes[2];                  // Falling boxes
RigidBody* floorRigidBody;      // Rigid body corresponding the floor


// Simulation function
void simulate() {

    // Update the physics simulation
    physicsEngine->update();

    // Display the scene
    display();
}

// Main function
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(600, 600);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("ReactPhysics3D Example - Falling Cubes");
    init();
    glutIdleFunc(simulate);
    glutReshapeFunc(reshape);
    glutMainLoop();
    physicsEngine->stop(); // Stop the physics simulation
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
    Vector3D gravity(0.0, -9.81, 0.0);

    // Create the physics world
    physicsWorld = new PhysicsWorld(gravity);

    // Create the physics engine with the previous physis world and a timestep
    // of 0.005 seconds
    physicsEngine = new PhysicsEngine(physicsWorld, 0.005);

    // Create a falling box with a size of 1m and weight of 3kg
    Vector3D position1(-2.0, 15.0, 0.0);                         // Position of the box
    Quaternion orientation(0.2, 1.0, 0.6, 0.0);                 // Orientation of the box
    boxes[0] = new Box(1.0, 3.0, position1, orientation);       // Creation of the box
    boxes[0]->getRigidBodyPointer()->setRestitution(0.5);       // How bouncy is the rigid body
    physicsWorld->addBody(boxes[0]->getRigidBodyPointer());     // Add the rigid body created in the constructor
                                                                // of the Box to the physics world


    // Create a second falling box with a size of 2m and weight of 4.5kg
    Vector3D position2 = Vector3D(2.0, 10.0, 0.0);               // Position of the box
    orientation = Quaternion(1.0, 1.0, 0.5, 0.0);               // Orientation of the box
    boxes[1] = new Box(2.0, 4.5, position2, orientation);       // Creation of the box
    boxes[1]->getRigidBodyPointer()->setRestitution(0.5);       // How bouncy is the rigid body
    physicsWorld->addBody(boxes[1]->getRigidBodyPointer());     // Add the rigid body created in the constructor
                                                                // of the Box to the physics world

    // Create the rigid body corresponding to the floor
    Vector3D positionFloor = Vector3D(0.0, 0.0, 0.0);
    orientation = Quaternion(0.0, 1.0, 0.0, 0.0);
    double mass = 100.0;

    // Local inertia tensor of a cube
    rp3d::Matrix3x3 inertiaTensor(1.0/12.0*mass*2*FLOOR_SIZE*FLOOR_SIZE, 0.0, 0.0,
                                  0.0, 1.0/12.0*mass*2*FLOOR_SIZE*FLOOR_SIZE, 0.0,
                                  0.0, 0.0, 1.0/12.0*mass*2*FLOOR_SIZE*FLOOR_SIZE);

    // Creation of the bounding volume for the collision
    // The bounding volume is an Oriented Bounding Box (OBB)
    // The first three arguments are the three axis direction of the OBB (here the x,y and z axis)
    // and the last three arguments are the corresponding half extents of the OBB in those direction.
    // Here the rigid body is a cube and therefore the three half extents are the half of the size
    // of the cube in all OBB directions.
    rp3d::OBB* boundingVolume = new OBB(positionFloor, Vector3D(1.0, 0.0, 0.0), Vector3D(0.0, 1.0, 0.0), Vector3D(0.0, 0.0, 1.0), FLOOR_SIZE, FLOOR_THICKNESS, FLOOR_SIZE);

    // Create the rigid body that will be used to simulate the physics of the box
    floorRigidBody = new RigidBody(positionFloor, orientation, mass, inertiaTensor, boundingVolume);

    // The floor is a rigid body that cannot move
    floorRigidBody->setIsMotionEnabled(false);

    // Set the bouncing restitution factor of the floor
    floorRigidBody->setRestitution(0.5);

    // Add the floor rigid body to the physics world
    physicsWorld->addBody(floorRigidBody);

    // Start the physics simulation
    physicsEngine->start();
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
    gluLookAt(20.0, 5.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

// Clean the memory allocation
void clean() {
    delete physicsEngine;
    delete physicsWorld;
    delete boxes[0];
    delete boxes[1];
    delete floorRigidBody;
}