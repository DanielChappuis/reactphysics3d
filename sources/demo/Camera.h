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
//Hellos
#ifndef CAMERA_H
#define CAMERA_H

// Libraries
#include "../reactphysics3d/reactphysics3d.h"       // We want the mathematics stuff of reactphysics3d

// Namespaces
using namespace reactphysics3d;

// Class Camera (abstract class)
// In the project we will use two different camera. This is the superclass of the two
// cameras. We will use a OutSideCamera that move arround the scene and an OnBoardCamera
// that will simulate the deplacement of a viewer inside the scene
class Camera {
    protected :
        Vector3D position;                                  // Position of the camera
        Vector3D lookAtPoint;                               // Point where the camera is looking
        Vector3D viewVector;                                // Vector from the camera position to the view point
		static double speed;                                // Speed movement of the camera

    public :
        Camera();                                           // Constructor
        Camera(const Camera& camera);                       // Copy-constructor
        virtual ~Camera();                                  // Destructor
        virtual Vector3D getPosition() const;               // Get the position of the camera
        virtual void setPosition(const Vector3D& pos);      // Set the position of the camera
        virtual Vector3D getLookAtPoint() const;            // Return the point where the camera is looking
        virtual Vector3D getViewVector() const;             // Return the view vector of the camera
        virtual void updateViewVector()=0;                  // Update the view vector of the camera
		static void increaseSpeed();                        // Increase the speed of camera movement
		static void decreaseSpeed();                        // Decrease the speed of camera movement
};

// Get the position of the camera (inline function)
inline Vector3D Camera::getPosition() const {
    // Return the position of the camera
    return position;
}

// Set the position of the camera (inline function)
inline void Camera::setPosition(const Vector3D& pos) {
    // Set the position of the camera
    position = pos;
}

// Return the point where the camera is looking (inline function)
inline Vector3D Camera::getLookAtPoint() const {
    return lookAtPoint;
}

// Return the view vector of the camera (inline function)
inline Vector3D Camera::getViewVector() const {
    return viewVector;
}

// Increase the speed movement of the camera (inline function)
inline void Camera::increaseSpeed()
{
	speed= speed+0.05;
	if(speed>1)
		speed=1;
}

// Decrease the speep movement of the camera (inline function)
inline void Camera::decreaseSpeed()
{
	speed = speed-0.05;
	if(speed<=0)
		speed=0.005;
}

#endif
