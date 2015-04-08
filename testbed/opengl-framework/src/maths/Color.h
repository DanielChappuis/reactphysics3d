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

#ifndef COLOR_H
#define COLOR_H

namespace openglframework {

// Structure Color
// This structure represents a RGBA color.
struct Color {

    public:

        // -------------------- Attributes -------------------- //

        // RGBA color components
        float r, g, b, a;

        // -------------------- Methods -------------------- //

        // Constructor
        Color() : r(1), g(1), b(1), a(1) {}

        // Constructor
        Color(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}

        // Copy-constructor
        Color(const Color& color) : r(color.r), g(color.g), b(color.b), a(color.a) {}

        // Destructor
        ~Color() {}

        // Return the black color
        static Color black() { return Color(0.0f, 0.0f, 0.0f, 1.0f);}

        // Return the white color
        static Color white() { return Color(1.0f, 1.0f, 1.0f, 1.0f);}

        // Return the red color
        static Color red() { return Color(1.0f, 0.0f, 0.0f, 1.0f);}

        // Return the green color
        static Color green() { return Color(0.0f, 1.0f, 0.0f, 1.0f);}

        // Return the blue color
        static Color blue() { return Color(0.0f, 0.0f, 1.0f, 1.0f);}

        // = operator
        Color& operator=(const Color& color) {
            if (&color != this) {
                r = color.r;
                g = color.g;
                b = color.b;
                a = color.a;
            }
            return *this;
        }
};

}

#endif
