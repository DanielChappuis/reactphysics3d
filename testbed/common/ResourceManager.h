/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com               *
* Copyright (c) 2010-2016 Daniel Chappuis                                     *
********************************************************************************
*                                                                              *
* This software is provided 'as-is', without any express or implied warranty. *
* In no event will the authors be held liable for any damages arising from the *
* use of this software.                                                        *
*                                                                              *
* Permission is granted to anyone to use this software for any purpose,       *
* including commercial applications, and to alter it and redistribute it       *
* freely, subject to the following restrictions:                              *
*                                                                              *
* 1. The origin of this software must not be misrepresented; you must not     *
*    claim that you wrote the original software. If you use this software     *
*    in a product, an acknowledgment in the product documentation would be    *
*    appreciated but is not required.                                         *
*                                                                              *
* 2. Altered source versions must be plainly marked as such, and must not be  *
*    misrepresented as being the original software.                           *
*                                                                              *
* 3. This notice may not be removed or altered from any source distribution.  *
*                                                                              *
********************************************************************************/

#ifndef RESOURCEMANAGER_H
#define RESOURCEMANAGER_H

// Libraries
#include <string>
#include <unordered_map>
#include <filesystem>

/**
 * @brief ResourceManager class for dynamic resource path discovery
 * 
 * This class provides a cross-platform solution for locating resource files
 * (shaders, meshes, etc.) dynamically, solving issues with hardcoded relative
 * paths that don't work across different build environments and compilers.
 */
class ResourceManager {

private:
    /// Cache for discovered resource paths
    static std::unordered_map<std::string, std::string> mResourcePaths;
    
    /// Base directory where resources are located
    static std::string mResourceBaseDir;
    
    /// Maximum number of parent directories to search
    static const int MAX_SEARCH_DEPTH = 4;
    
    /// Private constructor (singleton pattern)
    ResourceManager() = default;
    
    /// Find resource directory by name
    static std::string findResourceDirectory(const std::string& dirName);
    
    /// Initialize resource paths
    static void initialize();
    
    /// Flag to track if initialization has been performed
    static bool mIsInitialized;

public:
    
    /// Get the path for a shader file
    static std::string getShaderPath(const std::string& shaderFilename);
    
    /// Get the path for a mesh file
    static std::string getMeshPath(const std::string& meshFilename);
    
    /// Get the mesh directory path
    static std::string getMeshDirectoryPath();
    
    /// Check if a file exists at the given path
    static bool fileExists(const std::string& filepath);
    
    /// Set resource base directory manually (for testing or custom setups)
    static void setResourceBaseDirectory(const std::string& baseDir);
};

#endif // RESOURCEMANAGER_H