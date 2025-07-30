# Dynamic Resource File Location Fix - ReactPhysics3D Issue #416

## Problem Description

The ReactPhysics3D testbed application used hardcoded relative paths for resource files (shaders and .obj meshes), which caused issues when different compilers or IDEs placed the executable in different directories relative to the source tree. This particularly affected developers on Windows with Visual Studio Code and other build environments.

### Original Issues
- Hardcoded paths like `"shaders/depth.vert"`, `"meshes/castle.obj"` in multiple files
- Application would crash with "Cannot open file" errors when run from different build directories
- Different build systems (CMake, Visual Studio, etc.) place executables in varying directory structures

## Solution Overview

Implemented a comprehensive `ResourceManager` class that provides dynamic resource path discovery while maintaining full backwards compatibility.

## Implementation Details

### Core Components

#### 1. ResourceManager Class (`testbed/common/ResourceManager.h` & `.cpp`)

**Key Features:**
- **Dynamic Discovery**: Searches up to 4 parent directories from executable location
- **Multiple Search Patterns**: Looks for both `testbed/shaders` + `testbed/meshes` and direct `shaders` + `meshes`
- **Path Caching**: Caches discovered paths for performance optimization
- **Environment Override**: Supports `RP3D_RESOURCE_PATH` environment variable
- **Backwards Compatibility**: Falls back to relative paths if discovery fails
- **Cross-Platform**: Uses `std::filesystem` for proper path handling

**Public API:**
```cpp
static std::string getShaderPath(const std::string& shaderFilename);
static std::string getMeshPath(const std::string& meshFilename);
static std::string getMeshDirectoryPath();
static bool fileExists(const std::string& filepath);
static void setResourceBaseDirectory(const std::string& baseDir);
```

#### 2. Search Algorithm

1. Check for `RP3D_RESOURCE_PATH` environment variable
2. Start from current working directory
3. For each directory level (up to MAX_SEARCH_DEPTH=4):
   - Look for `testbed/shaders` and `testbed/meshes`
   - Look for direct `shaders` and `meshes` directories
   - Move up one parent directory if not found
4. Fallback to current directory with warning if nothing found

### Files Modified

#### Core Framework Files
- **`testbed/src/SceneDemo.cpp`**
  - Updated shader paths: `mDepthShader`, `mPhongShader`, `mColorShader`, `mQuadShader`
  - Updated mesh folder path: `mMeshFolderPath`
  - Added ResourceManager include

#### Common Object Files
- **`testbed/common/Dumbbell.cpp`** - Updated dumbbell.obj path
- **`testbed/common/Box.cpp`** - Updated cube.obj path  
- **`testbed/common/Capsule.cpp`** - Updated capsule.obj path
- **`testbed/common/Sphere.cpp`** - Updated sphere.obj path
- **`testbed/common/VisualContactPoint.cpp`** - Updated sphere.obj path

#### Scene Files
- **`testbed/scenes/concavemesh/ConcaveMeshScene.cpp`** - Updated castle.obj and convexmesh.obj paths
- **`testbed/scenes/collisiondetection/CollisionDetectionScene.cpp`** - Updated castle.obj and convexmesh.obj paths
- **`testbed/scenes/raycast/RaycastScene.cpp`** - Updated castle.obj and convexmesh.obj paths
- **`testbed/scenes/pile/PileScene.cpp`** - Updated convexmesh.obj path
- **`testbed/scenes/heightfield/HeightFieldScene.cpp`** - Updated convexmesh.obj path
- **`testbed/scenes/collisionshapes/CollisionShapesScene.cpp`** - Updated convexmesh.obj path

#### Build System
- **`testbed/CMakeLists.txt`** - Added ResourceManager.h and ResourceManager.cpp to COMMON_SOURCES

## Usage Examples

### Basic Usage
```cpp
// Old hardcoded approach
std::string shaderPath = "shaders/phong.vert";
std::string meshPath = meshFolderPath + "castle.obj";

// New dynamic approach
std::string shaderPath = ResourceManager::getShaderPath("phong.vert");
std::string meshPath = ResourceManager::getMeshPath("castle.obj");
```

### Environment Variable Override
```bash
# Set custom resource path
export RP3D_RESOURCE_PATH="/custom/path/to/resources"
./testbed
```

### Manual Override (for testing)
```cpp
ResourceManager::setResourceBaseDirectory("/custom/path");
```

## Benefits

1. **Cross-Platform Compatibility**: Works across different operating systems and build environments
2. **Build System Agnostic**: Compatible with CMake, Visual Studio, Code::Blocks, etc.
3. **Developer Friendly**: No manual path configuration required
4. **Backwards Compatible**: Existing setups continue to work unchanged
5. **Flexible**: Supports custom paths via environment variables
6. **Performance Optimized**: Caches discovered paths to avoid repeated filesystem searches
7. **Robust Error Handling**: Graceful fallback with informative warnings

## Testing Results

The implementation was successfully tested by:
1. Building the testbed application with CMake
2. Running from the build directory
3. Verifying dynamic resource discovery: "ResourceManager: Found resources at: /path/to/build/testbed"
4. Confirming all mesh and shader files load successfully
5. Application completed without crashes (exit code 0)

## Migration Notes

For developers extending the testbed:
- Replace hardcoded `"meshes/filename.obj"` with `ResourceManager::getMeshPath("filename.obj")`
- Replace hardcoded `"shaders/filename.ext"` with `ResourceManager::getShaderPath("filename.ext")`
- Replace hardcoded `"meshes/"` directory with `ResourceManager::getMeshDirectoryPath()`

## Future Enhancements

Potential improvements for future versions:
- Support for additional resource types (textures, sounds, etc.)
- Configuration file support
- Resource validation and integrity checking
- Resource hot-reloading for development

---

**Issue Reference**: ReactPhysics3D Issue #416 - Dynamic Resource File Location  
**Implementation Date**: January 2025  
**Status**: ✅ Complete and Tested