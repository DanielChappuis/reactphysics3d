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

#ifndef SHADER_H
#define SHADER_H

// Libraries
#include "definitions.h"
#include "maths/Matrix4.h"
#include "maths/Vector2.h"
#include "maths/Vector3.h"
#include "maths/Vector4.h"
#include <string>
#include <iostream>
#include <stdexcept>
#include <exception>
#include <nanogui/opengl.h>

namespace openglframework {

// Class Shader
class Shader {

    private :

        // -------------------- Attributes -------------------- //

        // Shader object program ID
        GLuint mProgramObjectID;

        // Filenames of the vertex and fragment shaders
        std::string mFilenameVertexShader, mFilenameFragmentShader;

    public :

        // -------------------- Methods -------------------- //

        // Constructor
        Shader();

        // Constructor with arguments
        Shader(const std::string vertexShaderFilename, const std::string fragmentShaderFilename);

        // Destructor
        ~Shader();

        // Create the shader
        bool create(const std::string vertexShaderFilename,
                    const std::string fragmentShaderFilename);

        // Clear the shader
        void destroy();

        // Bind the shader
        void bind() const;

        // Unbind the shader
        void unbind() const;

        // Return the location of a uniform variable inside a shader program
        GLint getUniformLocation(const std::string& variableName, bool errorIfMissing = true) const;

        // Return the location of an attribute variable inside a shader program
        GLint getAttribLocation(const std::string& variableName, bool errorIfMissing = true) const;

        // Set a float uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setFloatUniform(const std::string& variableName, float value, bool errorIfMissing = true) const;

        // Set an int uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setIntUniform(const std::string& variableName, int value, bool errorIfMissing = true) const;

		// Set an array of int uniform values to this shader (be careful if the uniform is not
		// used in the shader, the compiler will remove it, then when you will try
		// to set it, an assert will occur)
		void setIntArrayUniform(const std::string& variableName, GLint* values, int nbValues, bool errorIfMissing = true) const;

        // Set a vector 2 uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setVector2Uniform(const std::string& variableName, const Vector2& v, bool errorIfMissing = true) const;

        // Set a vector 3 uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setVector3Uniform(const std::string& variableName, const Vector3& v, bool errorIfMissing = true) const;

        // Set a vector 4 uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setVector4Uniform(const std::string& variableName, const Vector4 &v, bool errorIfMissing = true) const;

        // Set a 3x3 matrix uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setMatrix3x3Uniform(const std::string& variableName, const float* matrix,
                                 bool transpose = false, bool errorIfMissing = true) const;

        // Set a 3x3 matrix uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setMatrix3x3Uniform(const std::string& variableName, const Matrix3& matrix, bool errorIfMissing = true) const;

        // Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setMatrix4x4Uniform(const std::string& variableName, const float* matrix,
                                 bool transpose = false, bool errorIfMissing = true) const;

        // Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
        // used in the shader, the compiler will remove it, then when you will try
        // to set it, an assert will occur)
        void setMatrix4x4Uniform(const std::string& variableName, const Matrix4& matrix, bool errorIfMissing = true) const;

        // Return the shader object program ID
        GLuint getProgramObjectId() const;

        // Return true if the needed OpenGL extensions are available
        static bool checkOpenGLExtensions();
};

// Bind the shader
inline void Shader::bind() const {
    assert(mProgramObjectID != 0);
    glUseProgram(mProgramObjectID);
}

// Unbind the shader
inline void Shader::unbind() const {
    assert(mProgramObjectID != 0);
    glUseProgram(0);
}

// Return the location of a uniform variable inside a shader program
inline GLint Shader::getUniformLocation(const std::string& variableName, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = glGetUniformLocation(mProgramObjectID, variableName.c_str());
    if (location == -1 && errorIfMissing) {
        std::cerr << "Error in vertex shader " << mFilenameVertexShader << " or in fragment shader"
                  << mFilenameFragmentShader << " : No Uniform variable : " << variableName
                  << std::endl;
        //throw std::logic_error("Error in Shader");
    }

    return location;
}

// Return the location of an attribute variable inside a shader program
inline GLint Shader::getAttribLocation(const std::string& variableName, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = glGetAttribLocation(mProgramObjectID, variableName.c_str());
    if (location == -1 && errorIfMissing) {
        std::cerr << "Error in vertex shader " << mFilenameVertexShader << " or in fragment shader"
                  << mFilenameFragmentShader << " : No variable : " << variableName
                  << std::endl;
        throw std::logic_error("Error in Shader");
    }

    return location;
}

// Clear the shader
inline void Shader::destroy() {
    if (mProgramObjectID != 0) {
        glDeleteProgram(mProgramObjectID);
        mProgramObjectID = 0;
    }
}

// Set a float uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setFloatUniform(const std::string& variableName, float value, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform1f(location, value);
    }
}

// Set an int uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setIntUniform(const std::string& variableName, int value, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform1i(location, value);
    }
}

// Set an array of int uniform values to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setIntArrayUniform(const std::string& variableName, GLint* values, int nbValues, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform1iv(location, nbValues, values);
    }
}
// Set a vector 2 uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setVector2Uniform(const std::string& variableName, const Vector2& v, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform2f(location, v.x, v.y);
    }
}

// Set a vector 3 uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setVector3Uniform(const std::string& variableName, const Vector3 &v, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform3f(location, v.x, v.y, v.z);
    }
}

// Set a vector 4 uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setVector4Uniform(const std::string& variableName, const Vector4& v, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform4f(location, v.x, v.y, v.z, v.w);
    }
}

// Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setMatrix3x3Uniform(const std::string& variableName, const float* matrix,
                                        bool transpose, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniformMatrix3fv(location, 1, transpose, matrix);
    }
}

// Set a 3x3 matrix uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setMatrix3x3Uniform(const std::string& variableName, const Matrix3& matrix, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLfloat mat[9];
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            mat[i*3 + j] = matrix.getValue(i, j);
        }
    }
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniformMatrix3fv(location, 1, true, mat);
    }
}

// Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setMatrix4x4Uniform(const std::string& variableName, const float* matrix,
                                        bool transpose, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniformMatrix4fv(location, 1, transpose, matrix);
    }
}

// Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
// used in the shader, the compiler will remove it, then when you will try
// to set it, an assert will occur)
inline void Shader::setMatrix4x4Uniform(const std::string& variableName, const Matrix4& matrix, bool errorIfMissing) const {
    assert(mProgramObjectID != 0);
    GLfloat mat[16];
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            mat[i*4 + j] = matrix.m[i][j];
        }
    }
    GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniformMatrix4fv(location, 1, true, mat);
    }
}

// Return the shader object program ID
inline GLuint Shader::getProgramObjectId() const {
   return mProgramObjectID;
}

// Return true if the needed OpenGL extensions are available for shaders
inline bool Shader::checkOpenGLExtensions() {

    // Check that GLSL vertex and fragment shaders are available on the platform
    //return (GLEW_VERSION_2_0 || (GLEW_ARB_vertex_shader && GLEW_ARB_fragment_shader));
    return true;
}

}

#endif
