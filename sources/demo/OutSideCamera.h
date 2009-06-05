/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU Lesser General Public License as published *
 * by the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU Lesser General Public License for more details.                      *
 *                                                                          *
 * You should have received a copy of the GNU Lesser General Public License *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

#ifndef OUTSIDE_CAMERA_H
#define OUTSIDE_CAMERA_H

// Libraries
#include "Camera.h"

// ---- Class OutSideCamera ----
// This camera can be move everywhere around the scene to obtain a outside view of the house
class OutSideCamera : public Camera {
    private :
        double heightFromFloor;                                 // Height from the floor
        int horizontalAngleRotation;                            // Horizontal rotation angle (in degree)
        int verticalAngleRotation;                              // Vertical rotation angle (in degree)
        double distanceFromOrigin;                              // Distance of the camera from the origin (used to zoom)

    public :
        OutSideCamera();                                                    // Constructor
        OutSideCamera(const OutSideCamera& camera);                         // Copy-constructor
        virtual ~OutSideCamera();                                           // Destructor
        void updatePosition();                                              // Compute the new position of the camera
        void updateViewVector();                                            // Update the view vector of the camera
        double getHeightFromFloor() const;                                  // Get the height of the camera from the floor
        void setHeightFromFloor(double height);                             // Set the height of the camera from the floor
        void modifyHorizontalAngleRotation(int screenDistance, float fps);  // Modify the horizontal camera rotation angle
        void modifyVerticalAngleRotation(int screenDistance, float fps);    // Modify the vertical camera rotation angle
		void increaseDistance(float fps);                                   // Increase the distance of the camera from the origin
		void decreaseDistance(float fps);                                   // Decrease the distance of the camera from the origin
};

// Compute the new view vector of the camera (inline function)
inline void OutSideCamera::updateViewVector() {
    viewVector = Vector3D(0.0, 0.0, 0.0) - position;
}

// Get the height of the camera from the floor (inline function)
inline double OutSideCamera::getHeightFromFloor() const {
    return heightFromFloor;
}

// Set the height of the camera from the floor (inline function)
inline void OutSideCamera::setHeightFromFloor(double height) {
    heightFromFloor = height;
}

#endif
