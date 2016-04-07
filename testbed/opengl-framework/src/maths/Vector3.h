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

#ifndef OPENGLFRAMEWORK_VECTOR3_H
#define OPENGLFRAMEWORK_VECTOR3_H

// Libraries
#include <cmath>
#include <cassert>
#include <limits>

namespace openglframework {

// Class Vector3
// This class represents a 3D vector.
class Vector3 {

    public:

        // -------------------- Attributes -------------------- //

        // Components of the vector
        float x, y, z;

        // -------------------- Methods -------------------- //

        // Constructor
        Vector3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}

        // Constructor
        Vector3(const Vector3& vector) : x(vector.x), y(vector.y), z(vector.z) {}

        // Constructor
        ~Vector3() {}

        // = operator
        Vector3& operator=(const Vector3& vector) {
            if (&vector != this) {
                x = vector.x;
                y = vector.y;
                z = vector.z;
            }
            return *this;
        }

        // + operator
        Vector3 operator+(const Vector3 &v) const {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        // += operator
        Vector3& operator+=(const Vector3 &v) {
            x += v.x; y += v.y; z += v.z;
            return *this;
        }

        // - operator
        Vector3 operator-(const Vector3 &v) const {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        // -= operator
        Vector3& operator-=(const Vector3 &v) {
            x -= v.x; y -= v.y; z -= v.z;
            return *this;
        }

        // == operator
        bool operator==(const Vector3 &v) const {
            return x == v.x && y == v.y && z == v.z;
        }

        // != operator
        bool operator!=(const Vector3 &v) const {
          return !( *this == v );
        }

        // * operator
        Vector3 operator*(float f) const {
            return Vector3(f*x, f*y, f*z);
        }

        // *= operator
        Vector3 &operator*=(float f) {
            x *= f; y *= f; z *= f;
            return *this;
        }

        // / operator
        Vector3 operator/(float f) const {
            assert(f > std::numeric_limits<float>::epsilon() );
            float inv = 1.f / f;
            return Vector3(x * inv, y * inv, z * inv);
        }

        // /= operator
        Vector3 &operator/=(float f) {
            assert(f > std::numeric_limits<float>::epsilon());
            float inv = 1.f / f;
            x *= inv; y *= inv; z *= inv;
            return *this;
        }

        // - operator
        Vector3 operator-() const {
            return Vector3(-x, -y, -z);
        }

        // [] operator
        float &operator[](int i) {
            assert(i >= 0 && i <= 2);
            switch (i) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            }
            return z;
        }

        // [] operator
        const float &operator[](int i) const {
            assert(i >= 0 && i <= 2);
            switch (i) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            }
            return z;
        }

        // Cross product operator
        Vector3 cross(const Vector3 &v) const{
            return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
        }

        // Dot product operator
        float dot(const Vector3 &v) const{
            return x * v.x + y * v.y + z * v.z;
        }

        // Normalize the vector and return it
        Vector3 normalize() {
            float l = length();
            if(l < std::numeric_limits<float>::epsilon() ) {
              return *this;
              }
            x /= l;
            y /= l;
            z /= l;
            return *this;
        }

        bool isNull() const {
          return( x == 0. && y == 0. && z == 0. );
        }

        // Clamp the values between 0 and 1
        Vector3 clamp01() {
            if (x>1.f) x=1.f;
            else if (x<0.f) x=0.f;
            if (y>1.f) y=1.f;
            else if (y<0.f) y=0.f;
            if (z>1.f) z=1.f;
            else if (z<0.f) z=0.f;
            return *this;
        }

        // Return the squared length of the vector
        float lengthSquared() const { return x*x + y*y + z*z; }

        // Return the length of the vector
        float length() const { return sqrt(lengthSquared()); }
};

inline Vector3 operator*(float f, const Vector3 & o) {
  return o*f;
}

}

#endif
