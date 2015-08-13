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

#ifndef MATRIX4_H
#define MATRIX4_H

// Libraries
#include <cmath>
#include <assert.h>
#include <iostream>
#include "Vector3.h"
#include "Vector4.h"
#include "Matrix3.h"
#include "definitions.h"

namespace openglframework {

// Class Matrix4
// This class represents a 4x4 matrix
class Matrix4 {

    public:

        // -------------------- Attributes -------------------- //

        // Elements of the matrix
        float m[4][4];

        // -------------------- Methods -------------------- //

        // Constructor
        Matrix4(float m_00=0, float m_01=0, float m_02=0, float m_03=0,
                float m_10=0, float m_11=0, float m_12=0, float m_13=0,
                float m_20=0, float m_21=0, float m_22=0, float m_23=0,
                float m_30=0, float m_31=0, float m_32=0, float m_33=0) {
            m[0][0] = m_00; m[0][1] = m_01; m[0][2] = m_02;  m[0][3] = m_03;
            m[1][0] = m_10; m[1][1] = m_11; m[1][2] = m_12;  m[1][3] = m_13;
            m[2][0] = m_20; m[2][1] = m_21; m[2][2] = m_22;  m[2][3] = m_23;
            m[3][0] = m_30; m[3][1] = m_31; m[3][2] = m_32;  m[3][3] = m_33;
        }

        // Constructor
        Matrix4(float n[4][4]) {
            m[0][0]=n[0][0]; m[0][1]=n[0][1]; m[0][2]=n[0][2]; m[0][3]=n[0][3];
            m[1][0]=n[1][0]; m[1][1]=n[1][1]; m[1][2]=n[1][2]; m[1][3]=n[1][3];
            m[2][0]=n[2][0]; m[2][1]=n[2][1]; m[2][2]=n[2][2]; m[2][3]=n[2][3];
            m[3][0]=n[3][0]; m[3][1]=n[3][1]; m[3][2]=n[3][2]; m[3][3]=n[3][3];
        }

        // Constructor
        Matrix4(const Vector3& a1, const Vector3& a2, const Vector3& a3) {
            m[0][0] = a1.x; m[0][1] = a2.x; m[0][2] = a3.x;  m[0][3] = 0.f;
            m[1][0] = a1.y; m[1][1] = a2.y; m[1][2] = a3.y;  m[1][3] = 0.f;
            m[2][0] = a1.z; m[2][1] = a2.z; m[2][2] = a3.z;  m[2][3] = 0.f;
            m[3][0] = 0.f;  m[3][1] = 0.f;  m[3][2] = 0.f;   m[3][3] = 1.f;
        }

        // Constructor
        Matrix4(const Vector4& a1, const Vector4& a2, const Vector4& a3) {
            m[0][0] = a1.x; m[0][1] = a2.x; m[0][2] = a3.x;  m[0][3] = 0.f;
            m[1][0] = a1.y; m[1][1] = a2.y; m[1][2] = a3.y;  m[1][3] = 0.f;
            m[2][0] = a1.z; m[2][1] = a2.z; m[2][2] = a3.z;  m[2][3] = 0.f;
            m[3][0] = a1.w; m[3][1] = a2.w; m[3][2] = a3.w;  m[3][3] = 1.f;
        }

        // Constructor
        Matrix4(const Matrix4& matrix) {

            setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
                         matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
                         matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
                         matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
        }

        // + operator
        Matrix4 operator+(const Matrix4 &n) const {
            return Matrix4(m[0][0]+n.m[0][0], m[0][1]+n.m[0][1], m[0][2]+n.m[0][2], m[0][3]+n.m[0][3],
                           m[1][0]+n.m[1][0], m[1][1]+n.m[1][1], m[1][2]+n.m[1][2], m[1][3]+n.m[1][3],
                           m[2][0]+n.m[2][0], m[2][1]+n.m[2][1], m[2][2]+n.m[2][2], m[2][3]+n.m[2][3],
                           m[3][0]+n.m[3][0], m[3][1]+n.m[3][1], m[3][2]+n.m[3][2], m[3][3]+n.m[3][3]);
        }

        // += operator
        Matrix4& operator+=(const Matrix4 &n) {
            m[0][0]+=n.m[0][0]; m[0][1]+=n.m[0][1]; m[0][2]+=n.m[0][2]; m[0][3]+=n.m[0][3];
            m[1][0]+=n.m[1][0]; m[1][1]+=n.m[1][1]; m[1][2]+=n.m[1][2]; m[1][3]+=n.m[1][3];
            m[2][0]+=n.m[2][0]; m[2][1]+=n.m[2][1]; m[2][2]+=n.m[2][2]; m[2][3]+=n.m[2][3];
            m[3][0]+=n.m[3][0]; m[3][1]+=n.m[3][1]; m[3][2]+=n.m[3][2]; m[3][3]+=n.m[3][3];
            return *this;
        }

        // - operator
        Matrix4 operator-(const Matrix4 &n) const {
            return Matrix4(m[0][0]-n.m[0][0], m[0][1]-n.m[0][1], m[0][2]-n.m[0][2], m[0][3]-n.m[0][3],
                           m[1][0]-n.m[1][0], m[1][1]-n.m[1][1], m[1][2]-n.m[1][2], m[1][3]-n.m[1][3],
                           m[2][0]-n.m[2][0], m[2][1]-n.m[2][1], m[2][2]-n.m[2][2], m[2][3]-n.m[2][3],
                           m[3][0]-n.m[3][0], m[3][1]-n.m[3][1], m[3][2]-n.m[3][2], m[3][3]-n.m[3][3]);
        }

        // -= operator
        Matrix4& operator-=(const Matrix4 &n) {
            m[0][0]-=n.m[0][0]; m[0][1]-=n.m[0][1]; m[0][2]-=n.m[0][2]; m[0][3]-=n.m[0][3];
            m[1][0]-=n.m[1][0]; m[1][1]-=n.m[1][1]; m[1][2]-=n.m[1][2]; m[1][3]-=n.m[1][3];
            m[2][0]-=n.m[2][0]; m[2][1]-=n.m[2][1]; m[2][2]-=n.m[2][2]; m[2][3]-=n.m[2][3];
            m[3][0]-=n.m[3][0]; m[3][1]-=n.m[3][1]; m[3][2]-=n.m[3][2]; m[3][3]-=n.m[3][3];
            return *this;
        }

        // = operator
        Matrix4& operator=(const Matrix4& matrix) {
            if (&matrix != this) {
                setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
                             matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
                             matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
                             matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
            }
            return *this;
        }

        // == operator
        bool operator==(const Matrix4 &n) const {
            return m[0][0]==n.m[0][0] && m[0][1]==n.m[0][1] && m[0][2]==n.m[0][2] && m[0][3]==n.m[0][3] &&
                    m[1][0]==n.m[1][0] && m[1][1]==n.m[1][1] && m[1][2]==n.m[1][2] && m[1][3]==n.m[1][3] &&
                    m[2][0]==n.m[2][0] && m[2][1]==n.m[2][1] && m[2][2]==n.m[2][2] && m[2][3]==n.m[2][3] &&
                    m[3][0]==n.m[3][0] && m[3][1]==n.m[3][1] && m[3][2]==n.m[3][2] && m[3][3]==n.m[3][3];
        }

        // * operator
        Matrix4 operator*(const Matrix4 &n) const {
            Matrix4 o;
            for(int i = 0; i  < 4; i++) {
                for(int j = 0; j < 4; j++) {
                    float v = 0;
                    for(int k = 0; k < 4; k++) {
                        v += m[i][k] * n.m[k][j];
                    }
                    o.m[i][j] = v;
                }
            }
            return o;
        }

        // * operator
        Vector3 operator*(const Vector3 &v) const {
            Vector3 u =Vector3(m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3],
                               m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3],
                               m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]);
            float w = m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3];
            return u/w;
        }

        // * operator
        Vector4 operator*(const Vector4 &v) const {
            Vector4 u = Vector4(m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + v.w*m[0][3],
                                m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + v.w*m[1][3],
                                m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + v.w*m[2][3],
                                m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + v.w*m[3][3]);
            if(u.w != 0)
                return u/u.w;
            else
                return u;
        }

        // * operator
        Matrix4 operator*(float f) const {
            return Matrix4(m[0][0]*f, m[0][1]*f, m[0][2]*f,  m[0][3]*f,
                           m[1][0]*f, m[1][1]*f, m[1][2]*f,  m[1][3]*f,
                           m[2][0]*f, m[2][1]*f, m[2][2]*f,  m[2][3]*f,
                           m[3][0]*f, m[3][1]*f, m[3][2]*f,  m[3][3]*f);
        }

        // * operator
        Matrix4 &operator*=(float f) {
            m[0][0]*=f; m[0][1]*=f; m[0][2]*=f;  m[0][3]*=f;
            m[1][0]*=f; m[1][1]*=f; m[1][2]*=f;  m[1][3]*=f;
            m[2][0]*=f; m[2][1]*=f; m[2][2]*=f;  m[2][3]*=f;
            m[3][0]*=f; m[3][1]*=f; m[3][2]*=f;  m[3][3]*=f;
            return *this;
        }

        // / operator
        Matrix4 operator/(float f) const {
            assert(f!=0);
            return Matrix4(m[0][0]/f, m[0][1]/f, m[0][2]/f,  m[0][3]/f,
                           m[1][0]/f, m[1][1]/f, m[1][2]/f,  m[1][3]/f,
                           m[2][0]/f, m[2][1]/f, m[2][2]/f,  m[2][3]/f,
                           m[3][0]/f, m[3][1]/f, m[3][2]/f,  m[3][3]/f);
        }

        // /= operator
        Matrix4 &operator/=(float f) {
            assert(f!=0);
            m[0][0]/=f; m[0][1]/=f; m[0][2]/=f;  m[0][3]/=f;
            m[1][0]/=f; m[1][1]/=f; m[1][2]/=f;  m[1][3]/=f;
            m[2][0]/=f; m[2][1]/=f; m[2][2]/=f;  m[2][3]/=f;
            m[3][0]/=f; m[3][1]/=f; m[3][2]/=f;  m[3][3]/=f;
            return *this;
        }

        // - operator
        Matrix4 operator-() const {
            return Matrix4(-m[0][0], -m[0][1], -m[0][2],  -m[0][3],
                           -m[1][0], -m[1][1], -m[1][2],  -m[1][3],
                           -m[2][0], -m[2][1], -m[2][2],  -m[2][3],
                           -m[3][0], -m[3][1], -m[3][2],  -m[3][3]);
        }

        // Return the transpose matrix
        Matrix4 getTranspose() const {
            return Matrix4(m[0][0], m[1][0], m[2][0], m[3][0],
                           m[0][1], m[1][1], m[2][1], m[3][1],
                           m[0][2], m[1][2], m[2][2], m[3][2],
                           m[0][3], m[1][3], m[2][3], m[3][3]);
        }

        // Return the 3x3 upper-left matrix
        Matrix3 getUpperLeft3x3Matrix() const {
            return Matrix3(m[0][0], m[0][1], m[0][2],
                           m[1][0], m[1][1], m[1][2],
                           m[2][0], m[2][1], m[2][2]);
        }

        // Return the inversed matrix
        Matrix4 getInverse() const {
            int indxc[4], indxr[4];
            int ipiv[4] = { 0, 0, 0, 0 };
            float minv[4][4];
            float temp;

            for (int s=0; s<4; s++) {
                for (int t=0; t<4; t++) {
                    minv[s][t] = m[s][t];
                }
            }

            for (int i = 0; i < 4; i++) {
                int irow = -1, icol = -1;
                float big = 0.;
                // Choose pivot
                for (int j = 0; j < 4; j++) {
                    if (ipiv[j] != 1) {
                        for (int k = 0; k < 4; k++) {
                            if (ipiv[k] == 0) {
                                if (fabs(minv[j][k]) >= big) {
                                    big = float(fabs(minv[j][k]));
                                    irow = j;
                                    icol = k;
                                }
                            }
                            else if (ipiv[k] > 1) {
                                std::cout << "ERROR: Singular matrix in MatrixInvert\n";
                            }
                        }
                    }
                }
                ++ipiv[icol];
                // Swap rows _irow_ and _icol_ for pivot
                if (irow != icol) {
                    for (int k = 0; k < 4; ++k){
                        temp = minv[irow][k];
                        minv[irow][k] = minv[icol][k];
                        minv[icol][k] = temp;
                    }
                }
                indxr[i] = irow;
                indxc[i] = icol;
                if (minv[icol][icol] == 0.){
                    std::cout << "Singular matrix in MatrixInvert\n";
                }
                // Set $m[icol][icol]$ to one by scaling row _icol_ appropriately
                float pivinv = 1.f / minv[icol][icol];
                minv[icol][icol] = 1.f;
                for (int j = 0; j < 4; j++) {
                    minv[icol][j] *= pivinv;
                }

                // Subtract this row from others to zero out their columns
                for (int j = 0; j < 4; j++) {
                    if (j != icol) {
                        float save = minv[j][icol];
                        minv[j][icol] = 0;
                        for (int k = 0; k < 4; k++) {
                            minv[j][k] -= minv[icol][k]*save;
                        }
                    }
                }
            }
            // Swap columns to reflect permutation
            for (int j = 3; j >= 0; j--) {
                if (indxr[j] != indxc[j]) {
                    for (int k = 0; k < 4; k++){
                        temp = minv[k][indxr[j]];
                        minv[k][indxr[j]] = minv[k][indxc[j]];
                        minv[k][indxc[j]] = temp;
                    }
                }
            }
            return Matrix4(minv);
        }

        // Method to set all the values in the matrix
        void setAllValues(float a1, float a2, float a3, float a4,
                          float b1, float b2, float b3, float b4,
                          float c1, float c2, float c3, float c4,
                          float d1, float d2, float d3, float d4) {
            m[0][0] = a1; m[0][1] = a2; m[0][2] = a3, m[0][3] = a4;
            m[1][0] = b1; m[1][1] = b2; m[1][2] = b3; m[1][3] = b4;
            m[2][0] = c1; m[2][1] = c2; m[2][2] = c3; m[2][3] = c4;
            m[3][0] = d1; m[3][1] = d2; m[3][2] = d3; m[3][3] = d4;
        }

        // Set the matrix to the identity matrix
        Matrix4 setToIdentity() {
            m[0][0] = 1.f; m[0][1] = 0.f; m[0][2] = 0.f;  m[0][3] = 0.f;
            m[1][0] = 0.f; m[1][1] = 1.f; m[1][2] = 0.f;  m[1][3] = 0.f;
            m[2][0] = 0.f; m[2][1] = 0.f; m[2][2] = 1.f;  m[2][3] = 0.f;
            m[3][0] = 0.f; m[3][1] = 0.f; m[3][2] = 0.f;  m[3][3] = 1.f;
            return *this;
        }

        // Display the matrix
        void print() const {
            for (int i=0; i<4; i++) {
                for (int j=0; j<4; j++) {
                    std::cout << m[i][j];
                }
                std::cout << std::endl;
            }
        }

        // Return the pointer to the data array of the matrix
        float* dataBlock() {
            return m[0];
        }

        // Return the constant pointer to the data array of the matrix
        const float* dataBlock() const {
            return m[0];
        }

        // Return a given value from the matrix
        float getValue(int i, int j) const {
            assert(i >= 0 && i<4 && j >= 0 && j<4);
            return m[i][j];
        }

        // Return the trace of the matrix
        float getTrace() const {
            // Compute and return the trace
            return (m[0][0] + m[1][1] + m[2][2] + m[3][3]);
        }

        // Return a 4x4 translation matrix
        static Matrix4 translationMatrix(const Vector3& v);

        // Return a 4x4 rotation matrix
        static Matrix4 rotationMatrix(const Vector3& axis, float angle);

        // Return a 4x4 perspective projection matrix
        static Matrix4 perspectiveProjectionMatrix(float near, float far,
                                                   int width, int height,
                                                   float fieldOfView);

        // Return a 4x4 orthographic projection matrix
        static Matrix4 orthoProjectionMatrix(float near, float far, int width,
                                             int height);

        // Return the identity matrix
        static Matrix4 identity();
};

// * operator
inline Matrix4 operator*(float f, const Matrix4 & m) {
	return (m * f);
}

// Return a 4x4 translation matrix
inline Matrix4 Matrix4::translationMatrix(const Vector3& v) {
    return Matrix4(1, 0, 0, v.x,
                   0, 1, 0, v.y,
                   0, 0, 1, v.z,
                   0, 0, 0, 1);
}

// Return a 4x4 rotation matrix
inline Matrix4 Matrix4::rotationMatrix(const Vector3& axis, float angle) {

    float cosA = cos(angle);
    float sinA = sin(angle);
    Matrix4 rotationMatrix;
    rotationMatrix.setToIdentity();

    rotationMatrix.m[0][0] = cosA + (1-cosA) * axis.x * axis.x;
    rotationMatrix.m[0][1] = (1-cosA) * axis.x * axis.y - axis.z * sinA;
    rotationMatrix.m[0][2] = (1-cosA) * axis.x * axis.z + axis.y * sinA;
    rotationMatrix.m[0][3] = 0.f;

    rotationMatrix.m[1][0] = (1-cosA) * axis.x * axis.y + axis.z * sinA;
    rotationMatrix.m[1][1] = cosA + (1-cosA) * axis.y * axis.y;
    rotationMatrix.m[1][2] = (1-cosA) * axis.y * axis.z - axis.x * sinA;
    rotationMatrix.m[1][3] = 0.f;

    rotationMatrix.m[2][0] = (1-cosA) * axis.x * axis.z - axis.y * sinA;
    rotationMatrix.m[2][1] = (1-cosA) * axis.y * axis.z + axis.x * sinA;
    rotationMatrix.m[2][2] = cosA + (1-cosA) * axis.z * axis.z;
    rotationMatrix.m[2][3] = 0.f;

    rotationMatrix.m[3][0] = 0.f;
    rotationMatrix.m[3][1] = 0.f;
    rotationMatrix.m[3][2] = 0.f;
    rotationMatrix.m[3][3] = 1.f;

    return rotationMatrix;
}

// Return a 4x4 perspective projection matrix
inline Matrix4 Matrix4::perspectiveProjectionMatrix(float near, float far, int width, int height,
                                                    float fieldOfView) {

    // Compute the aspect ratio
    float aspect = float(width) / float(height);

    float top = near * tan((fieldOfView / 2.0f) * (float(PI) / 180.0f));
    float bottom = -top;
    float left = bottom * aspect;
    float right = top * aspect;

    float fx = 2.0f * near / (right - left);
    float fy = 2.0f * near / (top - bottom);
    float fz = -(far + near) / (far - near);
    float fw = -2.0f * far * near / (far - near);

    // Compute the projection matrix
    return Matrix4(fx, 0, 0, 0,
                   0, fy, 0, 0,
                   0, 0, fz, fw,
                   0, 0, -1, 0);
}

// Return a 4x4 orthographic projection matrix
inline Matrix4 Matrix4::orthoProjectionMatrix(float near, float far, int width, int height) {

    // Compute the aspect ratio
    float aspect = float(width) / float(height);

    float top = height * 0.5f;
    float bottom = -top;
    float left = bottom * aspect;
    float right = top * aspect;

    float fx = 2.0f / (right - left);
    float fy = 2.0f / (top - bottom);
    float fz = -2.0f / (far - near);
    float fw = -(far + near) / (far - near);

    // Compute the projection matrix
    return Matrix4(fx, 0, 0, 0,
                   0, fy, 0, 0,
                   0, 0, fz, fw,
                   0, 0, 0, 1);
}

// Return the identity matrix
inline Matrix4 Matrix4::identity() {
    return Matrix4(1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1);
}

}

#endif //_MATRIX4_H
