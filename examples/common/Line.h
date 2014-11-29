#ifndef LINE_H
#define LINE_H

// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"

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

        /// Update the transform matrix of the sphere
        void updateTransform();

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
