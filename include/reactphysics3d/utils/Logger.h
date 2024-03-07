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

#ifndef REACTPHYSICS3D_LOGGER_H
#define REACTPHYSICS3D_LOGGER_H

// Libraries
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/containers/Map.h>
#include <string>
#include <iostream>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Logger
/**
 * This abstract class is the base class used to log information, warnings or errors during the execution of the
 * library code for easier debugging.
 */
class Logger {

    public:

        /// Log verbosity levels
        enum class Level {Error = 1, Warning = 2, Information = 4};

        /// Log categories
        enum class Category {PhysicCommon, World, Body, Joint, Collider};

        /// Return the name of a category
        static std::string getCategoryName(Category category) {

            switch(category) {
                case Category::PhysicCommon: return "PhysicsCommon";
                case Category::World: return "World";
                case Category::Body: return "Body";
                case Category::Joint: return "Joint";
                case Category::Collider: return "Collider";
            }

            assert(false);
            return "";
        }

        /// Return the name of a level
        static std::string getLevelName(Level level) {

            switch(level) {
                case Level::Information: return "Information";
                case Level::Warning: return "Warning";
                case Level::Error: return "Error";
            }

            assert(false);
            return "";
        }

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Logger() = default;

        /// Destructor
        virtual ~Logger() = default;

        /// Log something
        virtual void log(Level level, const std::string& physicsWorldName, Category category, const std::string& message, const char* filename, int lineNumber)=0;

        // ---------- Friendship ---------- //

        friend class PhysicsCommon;
};

}

#endif
