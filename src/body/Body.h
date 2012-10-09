/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef BODY_H
#define BODY_H

// Libraries
#include <stdexcept>
#include <cassert>
#include "../configuration.h"

// Namespace reactphysics3d
namespace reactphysics3d {


/*  -------------------------------------------------------------------
    Class Body :
        This class is an abstract class to represent a body of the physics
        engine.
    -------------------------------------------------------------------
*/
class Body {

    protected :

        // -------------------- Attributes -------------------- //

        // ID of the body
        bodyindex mID;

        // -------------------- Methods -------------------- //

        // Private copy-constructor
        Body(const Body& body);

        // Private assignment operator
        Body& operator=(const Body& body);

    public :

        // -------------------- Methods -------------------- //

        // Constructor
        Body(bodyindex id);

        // Destructor
        virtual ~Body();

        // Return the id of the body
        bodyindex getID() const;

        // Smaller than operator
        bool operator<(const Body& body2) const;

        // Larger than operator
        bool operator>(const Body& body2) const;

        // Equal operator
        bool operator==(const Body& body2) const;

        // Not equal operator
        bool operator!=(const Body& body2) const;
};

// Return the id of the body
inline bodyindex Body::getID() const {
    return mID;
}

// Smaller than operator
inline bool Body::operator<(const Body& body2) const {
    return (mID < body2.mID);
} 

// Larger than operator
inline bool Body::operator>(const Body& body2) const {
    return (mID > body2.mID);
} 

// Equal operator
inline bool Body::operator==(const Body& body2) const {
    return (mID == body2.mID);
}
        
// Not equal operator
inline bool Body::operator!=(const Body& body2) const {
    return (mID != body2.mID);
}               

}

 #endif
