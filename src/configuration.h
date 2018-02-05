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

#ifndef REACTPHYSICS3D_CONFIGURATION_H
#define	REACTPHYSICS3D_CONFIGURATION_H

// Libraries
#include <limits>
#include <cfloat>
#include <utility>
#include <cstdint>
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

/// Default friction coefficient for a rigid body
constexpr decimal DEFAULT_FRICTION_COEFFICIENT = decimal(0.3);

/// Default bounciness factor for a rigid body
constexpr decimal DEFAULT_BOUNCINESS = decimal(0.5);

/// Default rolling resistance
constexpr decimal DEFAULT_ROLLING_RESISTANCE = decimal(0.0);

/// True if the sleeping technique is enabled
constexpr bool SLEEPING_ENABLED = true;

/// Distance threshold for two contact points for a valid persistent contact (in meters)
constexpr decimal PERSISTENT_CONTACT_DIST_THRESHOLD = decimal(0.03);

/// Velocity threshold for contact velocity restitution
constexpr decimal RESTITUTION_VELOCITY_THRESHOLD = decimal(1.0);

/// Number of iterations when solving the velocity constraints of the Sequential Impulse technique
constexpr uint DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS = 10;

/// Number of iterations when solving the position constraints of the Sequential Impulse technique
constexpr uint DEFAULT_POSITION_SOLVER_NB_ITERATIONS = 5;

/// Time (in seconds) that a body must stay still to be considered sleeping
constexpr float DEFAULT_TIME_BEFORE_SLEEP = 1.0f;

/// A body with a linear velocity smaller than the sleep linear velocity (in m/s)
/// might enter sleeping mode.
constexpr decimal DEFAULT_SLEEP_LINEAR_VELOCITY = decimal(0.02);

/// A body with angular velocity smaller than the sleep angular velocity (in rad/s)
/// might enter sleeping mode
constexpr decimal DEFAULT_SLEEP_ANGULAR_VELOCITY = decimal(3.0 * (PI / 180.0));

/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
/// inflated with a constant gap to allow the collision shape to move a little bit
/// without triggering a large modification of the tree which can be costly
constexpr decimal DYNAMIC_TREE_AABB_GAP = decimal(0.1);

/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
/// also inflated in direction of the linear motion of the body by mutliplying the
/// followin constant with the linear velocity and the elapsed time between two frames.
constexpr decimal DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER = decimal(1.7);

/// Maximum number of contact manifolds in an overlapping pair that involves two
/// convex collision shapes.
constexpr int NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE = 1;

/// Maximum number of contact manifolds in an overlapping pair that involves at
/// least one concave collision shape.
constexpr int NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE = 3;

/// This is used to test if two contact manifold are similar (same contact normal) in order to
/// merge them. If the cosine of the angle between the normals of the two manifold are larger
/// than the value bellow, the manifold are considered to be similar.
constexpr decimal COS_ANGLE_SIMILAR_CONTACT_MANIFOLD = decimal(0.95);

/// Size (in bytes) of the single frame allocator
constexpr size_t INIT_SINGLE_FRAME_ALLOCATOR_BYTES = 1048576; // 1Mb

}

#endif
