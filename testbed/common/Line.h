/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef LINE_H
#define LINE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>

// Class Line
class Line : public openglframework::Object3D {

    private :

        // -------------------- Attributes -------------------- //

        openglframework::Vector3 mWorldPoint1, mWorldPoint2;

        // -------------------- Methods -------------------- //

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Line(const openglframework::Vector3& worldPoint1,
             const openglframework::Vector3& worldPoint2);

        /// Destructor
        ~Line();

        /// Return the first point of the line
        openglframework::Vector3 getPoint1() const;

        /// Return the second point of the line
        openglframework::Vector3 getPoint2() const;

        /// Render the line at the correct position and with the correct orientation
        void render(openglframework::Shader& shader,
                    const openglframework::Matrix4& worldToCameraMatrix);
};

// Return the first point of the line
inline openglframework::Vector3 Line::getPoint1() const {
    return mWorldPoint1;
}

// Return the second point of the line
inline openglframework::Vector3 Line::getPoint2() const {
    return mWorldPoint2;
}

#endif
