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

#ifndef TEST_HEIGHT_FIELD_H
#define TEST_HEIGHT_FIELD_H

// Libraries
#include "Test.h"
#include <reactphysics3d/reactphysics3d.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestHeightField
/**
 * Unit test for the HeightField class.
 */
class TestHeightField : public Test {

    private :

        // ---------- Atributes ---------- //

        PhysicsCommon mPhysicsCommon;

        float mHeightData[2 * 3];
        HeightField* mHeightField;

        int mHeightDataScaled[2 * 3];
        HeightField* mHeightFieldScaled;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestHeightField(const std::string& name) : Test(name) {

            mHeightData[0] = 0.0; mHeightData[1] = 0.0;
            mHeightData[2] = 1.0; mHeightData[3] = 3.0;
            mHeightData[4] = 2.0; mHeightData[5] = 4.0;

            // Create a height-field with float values
            std::vector<rp3d::Message> messages;
            mHeightField = mPhysicsCommon.createHeightField(2, 3, mHeightData,
                                                            rp3d::HeightField::HeightDataType::HEIGHT_FLOAT_TYPE,
                                                            messages);
            rp3d_test(mHeightField != nullptr);

            mHeightDataScaled[0] = 0; mHeightDataScaled[1] = 0;
            mHeightDataScaled[2] = 1; mHeightDataScaled[3] = 3;
            mHeightDataScaled[4] = 2; mHeightDataScaled[5] = 4;

            // Create a height-field with integer values and integer scaling factor
            messages.clear();
            mHeightFieldScaled = mPhysicsCommon.createHeightField(2, 3, mHeightDataScaled,
                                                            rp3d::HeightField::HeightDataType::HEIGHT_INT_TYPE,
                                                            messages, 3.0);
            rp3d_test(mHeightFieldScaled != nullptr);
        }

        /// Destructor
        virtual ~TestHeightField() {

        }

        /// Run the tests
        void run() {
            testHeightField();
            testHeightFieldScaled();
        }

        void testHeightField() {

            // getNbRows() and getNbColumns()
            rp3d_test(mHeightField->getNbColumns() == 2);
            rp3d_test(mHeightField->getNbRows() == 3);

            // getMinHeight() and getMaxHeight()
            rp3d_test(mHeightField->getMinHeight() == 0);
            rp3d_test(mHeightField->getMaxHeight() == 4);

            // getIntegerHeightScale()
            rp3d_test(mHeightField->getIntegerHeightScale() == 1.0);

            // getHeightAt()
            rp3d_test(mHeightField->getHeightAt(0, 0) == 0.0);
            rp3d_test(mHeightField->getHeightAt(0, 1) == 1.0);
            rp3d_test(mHeightField->getHeightAt(0, 2) == 2.0);
            rp3d_test(mHeightField->getHeightAt(1, 0) == 0.0);
            rp3d_test(mHeightField->getHeightAt(1, 1) == 3.0);
            rp3d_test(mHeightField->getHeightAt(1, 2) == 4.0);

            // getBounds()
            rp3d_test(mHeightField->getHeightDataType() == rp3d::HeightField::HeightDataType::HEIGHT_FLOAT_TYPE);
            rp3d_test(Vector3::approxEqual(mHeightField->getBounds().getMin(), Vector3(-0.5, -2, -1)));
            rp3d_test(Vector3::approxEqual(mHeightField->getBounds().getMax(), Vector3(0.5, 2, 1)));

            // getVertexAt()
            rp3d_test(Vector3::approxEqual(mHeightField->getVertexAt(0, 0), Vector3(-0.5, -2, -1)));
            rp3d_test(Vector3::approxEqual(mHeightField->getVertexAt(0, 1), Vector3(-0.5, -1, 0)));
            rp3d_test(Vector3::approxEqual(mHeightField->getVertexAt(0, 2), Vector3(-0.5, 0, 1)));
            rp3d_test(Vector3::approxEqual(mHeightField->getVertexAt(1, 0), Vector3(0.5, -2, -1)));
            rp3d_test(Vector3::approxEqual(mHeightField->getVertexAt(1, 1), Vector3(0.5, 1, 0)));
            rp3d_test(Vector3::approxEqual(mHeightField->getVertexAt(1, 2), Vector3(0.5, 2, 1)));
        }

        void testHeightFieldScaled() {

            // getMinHeight() and getMaxHeight()
            rp3d_test(mHeightFieldScaled->getMinHeight() == 0);
            rp3d_test(mHeightFieldScaled->getMaxHeight() == 12);

            // getIntegerHeightScale()
            rp3d_test(mHeightFieldScaled->getIntegerHeightScale() == 3.0);

            // getHeightAt()
            rp3d_test(mHeightFieldScaled->getHeightAt(0, 0) == 0);
            rp3d_test(mHeightFieldScaled->getHeightAt(0, 1) == 3);
            rp3d_test(mHeightFieldScaled->getHeightAt(0, 2) == 6);
            rp3d_test(mHeightFieldScaled->getHeightAt(1, 0) == 0);
            rp3d_test(mHeightFieldScaled->getHeightAt(1, 1) == 9);
            rp3d_test(mHeightFieldScaled->getHeightAt(1, 2) == 12);

            // getBounds()
            rp3d_test(mHeightFieldScaled->getHeightDataType() == rp3d::HeightField::HeightDataType::HEIGHT_INT_TYPE);
            rp3d_test(Vector3::approxEqual(mHeightFieldScaled->getBounds().getMin(), Vector3(-0.5, -6, -1)));
            rp3d_test(Vector3::approxEqual(mHeightFieldScaled->getBounds().getMax(), Vector3(0.5, 6, 1)));

            // getVertexAt()
            rp3d_test(Vector3::approxEqual(mHeightFieldScaled->getVertexAt(0, 0), Vector3(-0.5, -6, -1)));
            rp3d_test(Vector3::approxEqual(mHeightFieldScaled->getVertexAt(0, 1), Vector3(-0.5, -3, 0)));
            rp3d_test(Vector3::approxEqual(mHeightFieldScaled->getVertexAt(0, 2), Vector3(-0.5, 0, 1)));
            rp3d_test(Vector3::approxEqual(mHeightFieldScaled->getVertexAt(1, 0), Vector3(0.5, -6, -1)));
            rp3d_test(Vector3::approxEqual(mHeightFieldScaled->getVertexAt(1, 1), Vector3(0.5, 3, 0)));
            rp3d_test(Vector3::approxEqual(mHeightFieldScaled->getVertexAt(1, 2), Vector3(0.5, 6, 1)));
        }
 };

}

#endif
