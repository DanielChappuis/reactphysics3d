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

// Libraries
#include "Shader.h"
#include <cassert>
#include <fstream>
#include <iostream>

// Namespaces
using namespace openglframework;
using namespace std;

// Constructor
Shader::Shader() : mProgramObjectID(0) {

}

// Constructor with arguments
Shader::Shader(const std::string vertexShaderFilename, const std::string fragmentShaderFilename)
    : mProgramObjectID(0) {

    // Create the shader
    create(vertexShaderFilename, fragmentShaderFilename);
}

// Destructor
Shader::~Shader() {
    destroy();
}

// Create the shader
bool Shader::create(const std::string vertexShaderFilename,
                    const std::string fragmentShaderFilename) {

    // Set the shader filenames
    mFilenameVertexShader = vertexShaderFilename;
    mFilenameFragmentShader = fragmentShaderFilename;

    // Check that the needed OpenGL extensions are available
    /*bool isExtensionOK = checkOpenGLExtensions();
    if (!isExtensionOK) {
       cerr << "Error : Impossible to use GLSL vertex or fragment shaders on this platform" << endl;
       assert(false);
       return false;
    }*/

    // Delete the current shader
    destroy();

    assert(!vertexShaderFilename.empty() && !fragmentShaderFilename.empty());

    // ------------------- Load the vertex shader ------------------- //
    GLuint vertexShaderID;
    std::ifstream fileVertexShader;
    fileVertexShader.open(vertexShaderFilename.c_str(), std::ios::binary);

    if (fileVertexShader.is_open()) {

        // Get the size of the file
        fileVertexShader.seekg(0, std::ios::end);
        uint fileSize = (uint) (fileVertexShader.tellg());
        assert(fileSize != 0);

        // Read the file
        fileVertexShader.seekg(std::ios::beg);
        char* bufferVertexShader = new char[fileSize + 1];
        fileVertexShader.read(bufferVertexShader, fileSize);
        fileVertexShader.close();
        bufferVertexShader[fileSize] = '\0';

        // Create the OpenGL vertex shader and compile it
        vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
        assert(vertexShaderID != 0);
        glShaderSource(vertexShaderID, 1, (const char **) (&bufferVertexShader), NULL);
        glCompileShader(vertexShaderID);
        delete[] bufferVertexShader;

        // Get the compilation information
        int compiled;
        glGetShaderiv(vertexShaderID, GL_COMPILE_STATUS, &compiled);

        // If the compilation failed
        if (compiled == 0) {

            // Get the log of the compilation
            int lengthLog;
            glGetShaderiv(vertexShaderID, GL_INFO_LOG_LENGTH, &lengthLog);
            char* str = new char[lengthLog];
            glGetShaderInfoLog(vertexShaderID, lengthLog, NULL, str);

            // Display the log of the compilation
            std::cerr << "Vertex Shader Error (in " << vertexShaderFilename << ") : " << str << std::endl;
            delete[] str;
            assert(false);
            return false;
        }
    }
    else {
        std::cerr << "Error : Impossible to open the vertex shader file " <<
                     vertexShaderFilename << std::endl;
        assert(false);
        return false;
    }

    // ------------------- Load the fragment shader ------------------- //
    GLuint fragmentShaderID;
    std::ifstream fileFragmentShader;
    fileFragmentShader.open(fragmentShaderFilename.c_str(), std::ios::binary);

    if (fileFragmentShader.is_open()) {

        // Get the size of the file
        fileFragmentShader.seekg(0, std::ios::end);
        uint fileSize = (uint) (fileFragmentShader.tellg());
        assert(fileSize != 0);

        // Read the file
        fileFragmentShader.seekg(std::ios::beg);
        char* bufferFragmentShader = new char[fileSize + 1];
        fileFragmentShader.read(bufferFragmentShader, fileSize);
        fileFragmentShader.close();
        bufferFragmentShader[fileSize] = '\0';

        // Create the OpenGL fragment shader and compile it
        fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
        assert(fragmentShaderID != 0);
        glShaderSource(fragmentShaderID, 1, (const char **) (&bufferFragmentShader), NULL);
        glCompileShader(fragmentShaderID);
        delete[] bufferFragmentShader;

        // Get the compilation information
        int compiled;
        glGetShaderiv(fragmentShaderID, GL_COMPILE_STATUS, &compiled);

        // If the compilation failed
        if (compiled == 0) {

            // Get the log of the compilation
            int lengthLog;
            glGetShaderiv(fragmentShaderID, GL_INFO_LOG_LENGTH, &lengthLog);
            char* str = new char[lengthLog];
            glGetShaderInfoLog(fragmentShaderID, lengthLog, NULL, str);

            // Display the log of the compilation
            std::cerr << "Fragment Shader Error (in " << fragmentShaderFilename << ") : " << str << std::endl;
            delete[] str;
            assert(false);
            return false;
        }
    }
    else {
        std::cerr << "Error : Impossible to open the fragment shader file " <<
                     fragmentShaderFilename << std::endl;
        assert(false);
        return false;
    }

    // Create the shader program and attach the shaders
    mProgramObjectID = glCreateProgram();
    assert(mProgramObjectID);
    glAttachShader(mProgramObjectID, vertexShaderID);
    glAttachShader(mProgramObjectID, fragmentShaderID);
    glDeleteShader(vertexShaderID);
    glDeleteShader(fragmentShaderID);

    // Try to link the program
    glLinkProgram(mProgramObjectID);
    int linked;
    glGetProgramiv(mProgramObjectID, GL_LINK_STATUS, &linked);
    if (!linked) {
        int logLength;
        glGetProgramiv(mProgramObjectID, GL_INFO_LOG_LENGTH, &logLength);
        char* strLog = new char[logLength];
        glGetProgramInfoLog(mProgramObjectID, logLength, NULL, strLog);
        cerr << "Linker Error in vertex shader " << vertexShaderFilename <<
                " or in fragment shader " << fragmentShaderFilename << " : " << strLog << endl;
        delete[] strLog;
        destroy();
        assert(false);
    }

    return true;
}
