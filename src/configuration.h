/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONFIGURATION_H
#define	REACTPHYSICS3D_CONFIGURATION_H

// Libraries
#include <limits>
#include <cfloat>
#include <utility>
#include <sstream>
#include <string>
#include "decimal.h"
#include "containers/Pair.h"

// Windows platform
#if defined(WIN32) ||defined(_WIN32) || defined(_WIN64) ||defined(__WIN32__) || defined(__WINDOWS__)
    #define WINDOWS_OS
#elif defined(__APPLE__)     // Apple platform
    #define APPLE_OS
#elif defined(__linux__) || defined(linux) || defined(__linux)    // Linux platform
    #define LINUX_OS
#endif

/// Namespace reactphysics3d
namespace reactphysics3d {

// ------------------- Type definitions ------------------- //

using uint = unsigned int;
using uchar = unsigned char;
using ushort = unsigned short;
using luint = long unsigned int;
using bodyindex = luint;
using bodyindexpair = Pair<bodyindex, bodyindex>;

using int8 = std::int8_t;
using uint8 = std::uint8_t;
using int16 = std::int16_t;
using uint16 = std::uint16_t;
using int32 = std::int32_t;
using uint32 = std::uint32_t;

// ------------------- Enumerations ------------------- //

/// Position correction technique used in the constraint solver (for joints).
/// BAUMGARTE_JOINTS : Faster but can be innacurate in some situations.
/// NON_LINEAR_GAUSS_SEIDEL : Slower but more precise. This is the option used by default.
enum class JointsPositionCorrectionTechnique {BAUMGARTE_JOINTS, NON_LINEAR_GAUSS_SEIDEL};

/// Position correction technique used in the contact solver (for contacts)
/// BAUMGARTE_CONTACTS : Faster but can be innacurate and can lead to unexpected bounciness
///                      in some situations (due to error correction factor being added to
///                      the bodies momentum).
/// SPLIT_IMPULSES : A bit slower but the error correction factor is not added to the
///                 bodies momentum. This is the option used by default.
enum class ContactsPositionCorrectionTechnique {BAUMGARTE_CONTACTS, SPLIT_IMPULSES};

// ------------------- Constants ------------------- //

/// Smallest decimal value (negative)
const decimal DECIMAL_SMALLEST = - std::numeric_limits<decimal>::max();

/// Maximum decimal value
const decimal DECIMAL_LARGEST = std::numeric_limits<decimal>::max();

/// Machine epsilon
const decimal MACHINE_EPSILON = std::numeric_limits<decimal>::epsilon();

/// Pi constant
constexpr decimal PI = decimal(3.14159265);

/// 2*Pi constant
constexpr decimal PI_TIMES_2 = decimal(6.28318530);

/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
/// inflated with a constant gap to allow the collision shape to move a little bit
/// without triggering a large modification of the tree which can be costly
constexpr decimal DYNAMIC_TREE_AABB_GAP = decimal(0.1);

/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
/// also inflated in direction of the linear motion of the body by mutliplying the
/// followin constant with the linear velocity and the elapsed time between two frames.
constexpr decimal DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER = decimal(1.7);

/// Current version of ReactPhysics3D
const std::string RP3D_VERSION = std::string("0.7.1");

/// Structure WorldSettings
/**
 * This class is used to describe some settings of a physics world.
 */
struct WorldSettings {

    /// Name of the world
    std::string worldName = "";

    /// Distance threshold for two contact points for a valid persistent contact (in meters)
    decimal persistentContactDistanceThreshold = decimal(0.03);

    /// Default friction coefficient for a rigid body
    decimal defaultFrictionCoefficient = decimal(0.3);

    /// Default bounciness factor for a rigid body
    decimal defaultBounciness = decimal(0.5);

    /// Velocity threshold for contact velocity restitution
    decimal restitutionVelocityThreshold = decimal(1.0);

    /// Default rolling resistance
    decimal defaultRollingRestistance = decimal(0.0);

    /// True if the sleeping technique is enabled
    bool isSleepingEnabled = true;

    /// Number of iterations when solving the velocity constraints of the Sequential Impulse technique
    uint defaultVelocitySolverNbIterations = 10;

    /// Number of iterations when solving the position constraints of the Sequential Impulse technique
    uint defaultPositionSolverNbIterations = 5;

    /// Time (in seconds) that a body must stay still to be considered sleeping
    float defaultTimeBeforeSleep = 1.0f;

    /// A body with a linear velocity smaller than the sleep linear velocity (in m/s)
    /// might enter sleeping mode.
    decimal defaultSleepLinearVelocity = decimal(0.02);

    /// A body with angular velocity smaller than the sleep angular velocity (in rad/s)
    /// might enter sleeping mode
    decimal defaultSleepAngularVelocity = decimal(3.0) * (PI / decimal(180.0));

    /// Maximum number of contact manifolds in an overlapping pair that involves two
    /// convex collision shapes.
    int nbMaxContactManifoldsConvexShape = 1;

    /// Maximum number of contact manifolds in an overlapping pair that involves at
    /// least one concave collision shape.
    int nbMaxContactManifoldsConcaveShape = 3;

    /// This is used to test if two contact manifold are similar (same contact normal) in order to
    /// merge them. If the cosine of the angle between the normals of the two manifold are larger
    /// than the value bellow, the manifold are considered to be similar.
    decimal cosAngleSimilarContactManifold = decimal(0.95);

    /// Return a string with the world settings
    std::string to_string() const {

        std::stringstream ss;

        ss << "worldName=" << worldName << std::endl;
        ss << "persistentContactDistanceThreshold=" << persistentContactDistanceThreshold << std::endl;
        ss << "defaultFrictionCoefficient=" << defaultFrictionCoefficient << std::endl;
        ss << "defaultBounciness=" << defaultBounciness << std::endl;
        ss << "restitutionVelocityThreshold=" << restitutionVelocityThreshold << std::endl;
        ss << "defaultRollingRestistance=" << defaultRollingRestistance << std::endl;
        ss << "isSleepingEnabled=" << isSleepingEnabled << std::endl;
        ss << "defaultVelocitySolverNbIterations=" << defaultVelocitySolverNbIterations << std::endl;
        ss << "defaultPositionSolverNbIterations=" << defaultPositionSolverNbIterations << std::endl;
        ss << "defaultTimeBeforeSleep=" << defaultTimeBeforeSleep << std::endl;
        ss << "defaultSleepLinearVelocity=" << defaultSleepLinearVelocity << std::endl;
        ss << "defaultSleepAngularVelocity=" << defaultSleepAngularVelocity << std::endl;
        ss << "nbMaxContactManifoldsConvexShape=" << nbMaxContactManifoldsConvexShape << std::endl;
        ss << "nbMaxContactManifoldsConcaveShape=" << nbMaxContactManifoldsConcaveShape << std::endl;
        ss << "cosAngleSimilarContactManifold=" << cosAngleSimilarContactManifold << std::endl;

        return ss.str();
    }
};

}

#endif
