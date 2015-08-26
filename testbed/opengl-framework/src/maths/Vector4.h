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

#ifndef VECTOR4_H
#define VECTOR4_H

// Libraries
#include <math.h>
#include <assert.h>

namespace openglframework {

// Class Vector4
// This class represents a 4D vector.
class Vector4 {

    public:

        // -------------------- Attributes -------------------- //

        // Components of the vector
        float x, y, z, w;

        // -------------------- Methods -------------------- //

        // Constructor
        Vector4(float x=0, float y=0, float z=0, float w=0) : x(x), y(y), z(z), w(w) {}

        // Constructor
        Vector4(const Vector4& vector) : x(vector.x), y(vector.y), z(vector.z), w(vector.w) {}

        // + operator
        Vector4 operator+(const Vector4 &v) const {
            return Vector4(x + v.x, y + v.y, z + v.z, w + v.w);
        }

        // += operator
        Vector4& operator+=(const Vector4 &v) {
            x += v.x; y += v.y; z += v.z; w += v.w;
            return *this;
        }

        // - operator
        Vector4 operator-(const Vector4 &v) const {
            return Vector4(x - v.x, y - v.y, z - v.z, w - v.w);
        }

        // -= operator
        Vector4& operator-=(const Vector4 &v) {
            x -= v.x; y -= v.y; z -= v.z, w -=v.w;
            return *this;
        }

        // = operator
        Vector4& operator=(const Vector4& vector) {
            if (&vector != this) {
                x = vector.x;
                y = vector.y;
                z = vector.z;
                w = vector.w;
            }
            return *this;
        }

        // == operator
        bool operator==(const Vector4 &v) const {
            return x == v.x && y == v.y && z == v.z && w == v.w;
        }

        // * operator
        Vector4 operator*(float f) const {
            return Vector4(f*x, f*y, f*z, f*w);
        }

        // *= operator
        Vector4 &operator*=(float f) {
            x *= f; y *= f; z *= f; w *= f;
            return *this;
        }

        // / operator
        Vector4 operator/(float f) const {
            assert(f!=0);
            float inv = 1.f / f;
            return Vector4(x * inv, y * inv, z * inv, w * inv);
        }

        // /= operator
        Vector4 &operator/=(float f) {
            assert(f!=0);
            float inv = 1.f / f;
            x *= inv; y *= inv; z *= inv; w *= inv;
            return *this;
        }

        // - operator
        Vector4 operator-() const {
            return Vector4(-x, -y, -z, -w);
        }

        // [] operator
        float &operator[](int i) {
            assert(i >= 0 && i <= 3);
            switch (i) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            case 3: return w;
            }
            return w;
        }

        // Dot product operator
        float dot(const Vector4 &v) const {
            return x * v.x + y * v.y + z * v.z + w * v.w;
        }

        // Multiply two vectors by their components
        Vector4 componentMul(const Vector4 &v) const {
            return Vector4(x * v.x, y * v.y, z * v.z, w * v.w);
        }

        // Clamp the values between 0 and 1
        Vector4 clamp01() {
            if (x>1.f) x=1.f;
            else if (x<0.f) x=0.f;
            if (y>1.f) y=1.f;
            else if (y<0.f) y=0.f;
            if (z>1.f) z=1.f;
            else if (z<0.f) z=0.f;
            if (w>1.f) w=1.f;
            else if (w<0.f) w=0.f;
            return *this;
        }

        // Return the squared length of the vector
        float lengthSquared() const { return x * x + y * y + z * z + w * w; }

        // Return the length of the vector
        float length() const { return sqrt(lengthSquared()); }
};

}

#endif //_VECTOR4_H
