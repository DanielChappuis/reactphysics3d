/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU Lesser General Public License as published *
 * by the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU Lesser General Public License for more details.                      *
 *                                                                          *
 * You should have received a copy of the GNU Lesser General Public License *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

// Librairies
#include "Scene.h"
#include "Objects.h"
#include <GL/gl.h>
#include <GL/glu.h>

// Constructor of the class Scene
Scene::Scene(rp3d::PhysicsWorld* world) {

    this->world = world;

    // Initialise the material specular color
    mat_specular[0] = 1.0;
    mat_specular[1] = 1.0;
    mat_specular[2] = 1.0;
    mat_specular[3] = 1.0;

    // Initialize the material shininess
    mat_shininess[0] = 50.0;

    // Initialise the light source position
    light_position[0] = 20.0;
    light_position[1] = 9.0;
    light_position[2] = 15.0;
    light_position[3] = 0.0;

    // Initialise the ambient color of the light
    ambient_color[0] = 1.0;
    ambient_color[1] = 1.0;
    ambient_color[2] = 1.0;
    ambient_color[3] = 0.7;

    // Initialise the diffuse light color
    white_light[0] = 1.0;
    white_light[1] = 1.0;
    white_light[2] = 1.0;
    white_light[3] = 1.0;
}

// Destructor of the class Scene
Scene::~Scene() {

}

// Init method
void Scene::init() {
    glClearColor(0.0, 0.0, 0.0, 0.0);           // Select the color for the background
    glShadeModel(GL_SMOOTH);
    glClearDepth(1.0);

    // Lighting settings
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);        // Specular color of the material
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);      // Shininess of the material
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);        // Position of the light source
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_color);          // Ambient color of the light
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);            // Diffuse color of the light
    glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);           // Specular color of the light

    glEnable(GL_LIGHTING);                                    // Activate the lighting
    glEnable(GL_LIGHT0);                                      // Activate a light source
    glEnable(GL_DEPTH_TEST);                                  // Activate the Depth buffer
    //glEnable(GL_CULL_FACE);
}

// Display method
void Scene::display(const Context& context) const {
    glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

    // Define the position and the direction of the camera
	//gluLookAt(30,10,0,0,0,0,0,1,0);
	double x = outsideCamera.getPosition().getX();
	double y = outsideCamera.getPosition().getY();
	double z = outsideCamera.getPosition().getZ();
    gluLookAt(x,y,z,0,0,0,0,1,0);

    // Draw all objects in the context
    for(int i=0; i<context.getNbObjects(); ++i)
    {

        // Copy the active matrix on the matrix stack
        glPushMatrix();

        // Draw the object
        context.getObject(i).draw();

       glPopMatrix();
       glPushMatrix();

       // Draw the bounding volume
       context.getObject(i).getRigidBody()->getNarrowBoundingVolume()->draw();

       // Remove the matrix on the top of the matrix stack
       glPopMatrix();

       
    }

    // Draw all the contact points
    for (std::vector<Constraint*>::iterator it = world->getConstraintsBeginIterator(); it != world->getConstraintsEndIterator(); ++it) {
        RigidBody* rigidBody1 = dynamic_cast<RigidBody*>((*it)->getBody1());
        RigidBody* rigidBody2 = dynamic_cast<RigidBody*>((*it)->getBody2());
        //rigidBody1->setIsMotionEnabled(false);
        //rigidBody2->setIsMotionEnabled(false);

        Contact* contact = dynamic_cast<Contact*>((*it));
        assert(contact != 0);

        // Draw the contact points
        glPushMatrix();
        glTranslatef(contact->getPoint().getX(), contact->getPoint().getY(), contact->getPoint().getZ());
        contact->draw();
        glPopMatrix();
    }

    glFlush();

    // Swap the buffers
    SDL_GL_SwapBuffers();
}

// Reshape the window
void Scene::reshape(int width, int height) {
    glViewport(0,0,width,height);
	glMatrixMode(GL_PROJECTION);                           // Specify the matrix that will be modified
	glLoadIdentity();                                      // Load the identity matrix before the transformations
	gluPerspective(45.0, (float) width/height, 0.1f, 150.0f);
	glMatrixMode(GL_MODELVIEW);                            // Specify the matrix that will be modified
	glLoadIdentity();                                      // Load the identity matrix before the transformations
}
