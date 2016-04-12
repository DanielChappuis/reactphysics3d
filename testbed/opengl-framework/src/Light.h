/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2013 Daniel Chappuis                                            *
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

#ifndef LIGHT_H
#define LIGHT_H

// Libraries
#include "maths/Color.h"
#include "maths/Vector3.h"
#include "Object3D.h"
#include "Texture2D.h"
#include "Shader.h"
#include <nanogui/opengl.h>

namespace openglframework {

// Class Light
class Light : public Object3D {

    private:

        // -------------------- Attributes -------------------- //

        // Diffuse color of the light
        Color mDiffuseColor;

        // Specular color of the light
        Color mSpecularColor;

        // True if the light is active
        bool mIsActive;

    public:

        // -------------------- Methods -------------------- //

        // Constructor
        Light();

        // Constructor
        Light(Color diffuseColor, Color specularColor);

        // Destructor
        virtual ~Light();

        // Return the diffuse color
        Color getDiffuseColor() const;

        // Set the diffuse color
        void setDiffuseColor(const Color& color);

        // Return the specular color
        Color getSpecularColor() const;

        // Set the specular color
        void setSpecularColor(const Color& color);

        // Return true if the light is active
        bool getIsActive() const;

        // Initialize the light
        void init();

        // Enable the light
        void enable();

        // Disable the light
        void disable();
};

// Return the diffuse color
inline Color Light::getDiffuseColor() const {
    return mDiffuseColor;
}

// Set the diffuse color
inline void Light::setDiffuseColor(const Color& color) {
    mDiffuseColor = color;
}

// Return the specular color
inline Color Light::getSpecularColor() const {
    return mSpecularColor;
}

// Set the specular color
inline void Light::setSpecularColor(const Color& color) {
    mSpecularColor = color;
}

// Return true if the light is active
inline bool Light::getIsActive() const {
    return mIsActive;
}

// Enable the light
inline void Light::enable() {

    mIsActive = true;

    // Enable the light
    //glEnable(GL_LIGHTING);
    //glEnable(GL_LIGHT0 + mLightID);
}

// Disable the light
inline void Light::disable() {

    mIsActive = false;

    // Disable the light
    //glDisable(GL_LIGHT0 + mLightID);
}

}

#endif
