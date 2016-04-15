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

#ifndef VECTOR2_H
#define VECTOR2_H

// Libraries
#include <math.h>
#include <assert.h>

namespace openglframework {

// Class Vector2
// This class represents a 2D vector.
class Vector2 {

    public:

        // -------------------- Attributes -------------------- //

        // Components of the vector
        float x, y;


        // -------------------- Methods -------------------- //

        // Constructor
        Vector2(float x=0, float y=0) : x(x), y(y) {}

        // Constructor
        Vector2(const Vector2& vector) : x(vector.x), y(vector.y) {}

        // + operator
        Vector2 operator+(const Vector2 &v) const {
            return Vector2(x + v.x, y + v.y);
        }

        // += operator
        Vector2& operator+=(const Vector2 &v) {
            x += v.x; y += v.y;
            return *this;
        }

        // - operator
        Vector2 operator-(const Vector2 &v) const {
            return Vector2(x - v.x, y - v.y);
        }

        // -= operator
        Vector2& operator-=(const Vector2 &v) {
            x -= v.x; y -= v.y;
            return *this;
        }

        // = operator
        Vector2& operator=(const Vector2& vector) {
            if (&vector != this) {
                x = vector.x;
                y = vector.y;
            }
            return *this;
        }

        // == operator
        bool operator==(const Vector2 &v) const {
            return x == v.x && y == v.y;
        }

        // * operator
        Vector2 operator*(float f) const {
            return Vector2(f*x, f*y);
        }

        // *= operator
        Vector2 &operator*=(float f) {
            x *= f; y *= f;
            return *this;
        }

        // / operator
        Vector2 operator/(float f) const {
            assert(f!=0);
            float inv = 1.f / f;
            return Vector2(x * inv, y * inv);
        }

        // /= operator
        Vector2 &operator/=(float f) {
            assert(f!=0);
            float inv = 1.f / f;
            x *= inv; y *= inv;
            return *this;
        }

        // - operator
        Vector2 operator-() const {
            return Vector2(-x, -y);
        }

        // [] operator
        float &operator[](int i) {
            assert(i >= 0 && i <= 1);
            switch (i) {
                case 0: return x;
                case 1: return y;
            }
            return y;
        }

        // Normalize the vector and return it
        Vector2 normalize() {
            float l = length();
            assert(l > 0);
            x /= l;
            y /= l;
            return *this;
        }

        // Clamp the vector values between 0 and 1
        Vector2 clamp01() {
            if (x>1.f) x=1.f;
            else if (x<0.f) x=0.f;
            if (y>1.f) y=1.f;
            else if (y<0.f) y=0.f;
            return *this;
        }

        // Return the squared length of the vector
        float lengthSquared() const { return x*x + y*y; }

        // Return the length of the vector
        float length() const { return sqrt(lengthSquared()); }
};

}

#endif
