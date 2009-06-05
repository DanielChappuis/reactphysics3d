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

// Libraries
#include "OutSideCamera.h"
#include <cmath>

// Constructor
OutSideCamera::OutSideCamera() {

    // Initialize the attributes
    heightFromFloor = 20.0;
    horizontalAngleRotation = 0;
    verticalAngleRotation = 45;
    distanceFromOrigin = 40.0;
    lookAtPoint.setAllValues(0.0, 0.0, 0.0);

    // Update the position of the camera
    updatePosition();

    // Update the view vector of the camera
    updateViewVector();
}

// Destructor
OutSideCamera::~OutSideCamera() {

}

// Compute the new position of the camera
void OutSideCamera::updatePosition() {

    // Compute the floor distance from origin
    double floorDistance = distanceFromOrigin * cos(PI/180.0 * verticalAngleRotation);

    // Update the position of the camera
    position.setAllValues(floorDistance*cos(PI/180.0 * horizontalAngleRotation), distanceFromOrigin*sin(PI/180*verticalAngleRotation),
                          floorDistance*sin(PI/180.0 * horizontalAngleRotation));
}

// Set the camera rotation angle and update the position of the camera
void OutSideCamera::modifyHorizontalAngleRotation(int screenDistance, float fps) {

    // Update the horizontal rotation angle of the camera
    horizontalAngleRotation = (horizontalAngleRotation + int(screenDistance * 700.0 / fps)) % 360;

    // Update the position and the view vector of the camera
    updatePosition();
    updateViewVector();
}

// Set the vertical camera rotation angle
void OutSideCamera::modifyVerticalAngleRotation(int screenDistance, float fps) {

    // Update the vertical rotation angle of the camera
    verticalAngleRotation = verticalAngleRotation + (screenDistance * 700.0 / fps);

    // Vertical angle limits
    if (verticalAngleRotation > 89) {
        verticalAngleRotation = 89;
    }
    if (verticalAngleRotation < 1) {
        verticalAngleRotation = 1;
    }

    // Update the position and the view vector of the camera
    updatePosition();
    updateViewVector();
}

// Increase the distance from origine of the camera (used for the zoom)
void OutSideCamera::increaseDistance(float fps) {

    // Increase the distance from origin
	distanceFromOrigin = distanceFromOrigin + (speed * 60 / fps);

	// Update the position and the view vector of the camera
	updatePosition();
	updateViewVector();
}

// Decrease the distance from origine of the camera (used for the zoom)
void OutSideCamera::decreaseDistance(float fps) {

    // Decrease the distance from origin
	distanceFromOrigin = distanceFromOrigin - (speed * 60 / fps);

	// Limit condition
	if(distanceFromOrigin < 1) {
		distanceFromOrigin=1;
    }

	 // Update the position and the view vector of the camera
	 updatePosition();
	 updateViewVector();
}
