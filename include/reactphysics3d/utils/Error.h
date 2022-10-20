/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_ERROR_H
#define REACTPHYSICS3D_ERROR_H

// Libraries
#include <string>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Structure Error
/**
 * This structure represent an error that can be returned to the user
 */
struct Error {

    public:

        /// Error level
        enum class Level {Error = 1, Warning = 2, Information = 4};

        /// Error message
        std::string message;

        // Level (error, warning, information)
        Level level;

        // -------------------- Methods -------------------- //

        /// Constructor
        Error(std::string message, Level level = Level::Error) : message(message), level(level) {

        }
};

}

#endif

