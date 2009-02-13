/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by     *
 * the Free Software Foundation, either version 3 of the License, or        *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU General Public License for more details.                             *
 *                                                                          *
 * You should have received a copy of the GNU General Public License        *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

#ifndef SCENE_H
#define SCENE_H

// Libraries
#include "Context.h"
#include "OutSideCamera.h"
#include <SDL/SDL.h>
#include <GL/freeglut.h>        // Used only to draw cubes
#include <GL/gl.h>
#include <GL/glu.h>


// Scene class
class Scene {
    private :
        GLfloat mat_specular[4];                         // Material specular light color
        GLfloat mat_shininess[1];                        // Material shininess
        GLfloat light_position[4];                       // Position of the light source
        GLfloat ambient_color[4];                        // Ambient color of the light
        GLfloat white_light[4];                          // White light color
        OutSideCamera outsideCamera;                     // OutSide camera (Camera that can move around the scene)

    public  :
        Scene();                                        // constructor of the class
        ~Scene();                                       // Destructor of the class
        void init();                                    // Initialize the values of OpenGL
        void display(const Context& context) const;     // display the scene
        void reshape(int width, int height);            // Reshape the window
        OutSideCamera& getOutSideCamera() ;             // Return a reference to the outside camera
        float getCameraAngle1() const;                  // Return the angle of the camera
        float getCameraAngle2() const;                  // Return the angle of the camera
        void setCameraAngle1(float angle);              // Set the angle of the camera
        void setCameraAngle2(float angle);              // Set the angle of the camera
};

// Return a reference to the camera
inline OutSideCamera& Scene::getOutSideCamera() {
    return outsideCamera;
}

#endif
