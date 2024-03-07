/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_MESSAGE_H
#define REACTPHYSICS3D_MESSAGE_H

// Libraries
#include <string>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Structure Message
/**
 * This structure represent a message that can be returned to the user
 */
struct Message {

    public:

        /// Type of message
        enum class Type {Error = 1, Warning = 2, Information = 4};

        /// Message text
        std::string text;

        // Type (error, warning, information)
        Type type;

        // -------------------- Methods -------------------- //

        /// Constructor
        Message(std::string text, Type type = Type::Error) : text(text), type(type) {

        }
};

}

#endif

