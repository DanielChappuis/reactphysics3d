#ifndef SCENE_H
#define SCENE_H

// Libraries
#include "Context.h"

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
        float camera_angle1;                             // Camera angle
        float camera_angle2;                             // Camera angle

    public  :
        Scene();                                        // constructor of the class
        ~Scene();                                       // Destructor of the class
        void init();                                    // Initialize the values of OpenGL
        void display(const Context& context) const;     // display the scene
        void reshape(int width, int height);            // Reshape the window
        float getCameraAngle1() const;                  // Return the angle of the camera
        float getCameraAngle2() const;                  // Return the angle of the camera
        void setCameraAngle1(float angle);              // Set the angle of the camera
        void setCameraAngle2(float angle);              // Set the angle of the camera
};

// Return the angle of the camera (inline)
inline float Scene::getCameraAngle1() const {
    return camera_angle1;
}

// Return the angle of the camera (inline)
inline float Scene::getCameraAngle2() const {
    return camera_angle2;
}

// Set the angle of the camera (inline)
inline void Scene::setCameraAngle1(float angle) {
    camera_angle1 = angle;
}

// Set the angle of the camera (inline)
inline void Scene::setCameraAngle2(float angle) {
    camera_angle2 = angle;
}

#endif
