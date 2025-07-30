/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com               *
* Copyright (c) 2010-2016 Daniel Chappuis                                     *
********************************************************************************/

// Libraries
#include "ResourceManager.h"
#include <iostream>
#include <cstdlib>

// Static member initialization
std::unordered_map<std::string, std::string> ResourceManager::mResourcePaths;
std::string ResourceManager::mResourceBaseDir;
bool ResourceManager::mIsInitialized = false;

// Initialize resource paths
void ResourceManager::initialize() {
    
    if (mIsInitialized) return;
    
    // Check for environment variable override first
    const char* envPath = std::getenv("RP3D_RESOURCE_PATH");
    if (envPath != nullptr) {
        mResourceBaseDir = std::string(envPath);
        std::cout << "ResourceManager: Using environment path: " << mResourceBaseDir << std::endl;
    } else {
        // Dynamic discovery starting from executable directory
        std::filesystem::path currentPath = std::filesystem::current_path();
        
        // Search for testbed directory structure
        bool found = false;
        std::filesystem::path searchPath = currentPath;
        
        for (int depth = 0; depth <= MAX_SEARCH_DEPTH && !found; ++depth) {
            
            // Look for testbed/shaders and testbed/meshes directories
            std::filesystem::path testbedPath = searchPath / "testbed";
            std::filesystem::path shadersPath = testbedPath / "shaders";
            std::filesystem::path meshesPath = testbedPath / "meshes";
            
            if (std::filesystem::exists(shadersPath) && std::filesystem::exists(meshesPath)) {
                mResourceBaseDir = testbedPath.string();
                found = true;
                std::cout << "ResourceManager: Found resources at: " << mResourceBaseDir << std::endl;
            } else {
                // Try direct shaders/meshes in current search path
                shadersPath = searchPath / "shaders";
                meshesPath = searchPath / "meshes";
                
                if (std::filesystem::exists(shadersPath) && std::filesystem::exists(meshesPath)) {
                    mResourceBaseDir = searchPath.string();
                    found = true;
                    std::cout << "ResourceManager: Found resources at: " << mResourceBaseDir << std::endl;
                }
            }
            
            // Move up one directory level
            if (!found && searchPath.has_parent_path()) {
                searchPath = searchPath.parent_path();
            }
        }
        
        if (!found) {
            // Fallback to current directory (maintains backwards compatibility)
            mResourceBaseDir = currentPath.string();
            std::cout << "ResourceManager: Warning - Could not locate resource directories. "
                      << "Using current directory: " << mResourceBaseDir << std::endl;
            std::cout << "ResourceManager: You can set RP3D_RESOURCE_PATH environment variable "
                      << "to specify the resource directory manually." << std::endl;
        }
    }
    
    // Cache common directory paths
    std::filesystem::path basePath(mResourceBaseDir);
    mResourcePaths["shaders"] = (basePath / "shaders").string();
    mResourcePaths["meshes"] = (basePath / "meshes").string();
    
    mIsInitialized = true;
}

// Get shader file path
std::string ResourceManager::getShaderPath(const std::string& shaderFilename) {
    initialize();
    
    std::filesystem::path shaderPath = std::filesystem::path(mResourcePaths["shaders"]) / shaderFilename;
    std::string fullPath = shaderPath.string();
    
    // Check if file exists, fallback to relative path for backwards compatibility
    if (!std::filesystem::exists(fullPath)) {
        std::cout << "ResourceManager: Warning - Shader not found at: " << fullPath 
                  << ". Falling back to relative path." << std::endl;
        return "shaders/" + shaderFilename;
    }
    
    return fullPath;
}

// Get mesh file path
std::string ResourceManager::getMeshPath(const std::string& meshFilename) {
    initialize();
    
    std::filesystem::path meshPath = std::filesystem::path(mResourcePaths["meshes"]) / meshFilename;
    std::string fullPath = meshPath.string();
    
    // Check if file exists, fallback to relative path for backwards compatibility
    if (!std::filesystem::exists(fullPath)) {
        std::cout << "ResourceManager: Warning - Mesh not found at: " << fullPath 
                  << ". Falling back to relative path." << std::endl;
        return "meshes/" + meshFilename;
    }
    
    return fullPath;
}

// Get mesh directory path
std::string ResourceManager::getMeshDirectoryPath() {
    initialize();
    
    std::string meshDir = mResourcePaths["meshes"] + "/";
    
    // Check if directory exists, fallback for backwards compatibility
    if (!std::filesystem::exists(mResourcePaths["meshes"])) {
        std::cout << "ResourceManager: Warning - Mesh directory not found at: " << mResourcePaths["meshes"] 
                  << ". Falling back to relative path." << std::endl;
        return "meshes/";
    }
    
    return meshDir;
}

// Check if file exists
bool ResourceManager::fileExists(const std::string& filepath) {
    return std::filesystem::exists(filepath);
}

// Set resource base directory manually
void ResourceManager::setResourceBaseDirectory(const std::string& baseDir) {
    mResourceBaseDir = baseDir;
    mIsInitialized = false; // Force re-initialization
    initialize();
}