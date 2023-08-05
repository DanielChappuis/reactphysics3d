/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <cstdint>
#include <limits>
#include <cfloat>
#include <utility>
#include <sstream>
#include <string>
#include <reactphysics3d/decimal.h>
#include <reactphysics3d/containers/Pair.h>

// Compilers
#if defined(_MSC_VER)
    #define RP3D_COMPILER_VISUAL_STUDIO
#elif defined(__clang__)
    #define RP3D_COMPILER_CLANG
#elif defined(__GNUC__)
    #define RP3D_COMPILER_GCC
#else
    #define RP3D_COMPILER_UNKNOWN
#endif

// Force RP3D_FORCE_INLINE macro
#if defined(RP3D_COMPILER_VISUAL_STUDIO)
    #define RP3D_FORCE_INLINE __forceinline
#elif defined(RP3D_COMPILER_GCC) || defined(RP3D_COMPILER_CLANG)
    #define RP3D_FORCE_INLINE inline __attribute__((always_inline))
#else
    #define RP3D_FORCE_INLINE inline
#endif

/// Namespace reactphysics3d
namespace reactphysics3d {

// ------------------- Type definitions ------------------- //

using uint = unsigned int;
using uchar = unsigned char;
using ushort = unsigned short;
using luint = long unsigned int;

using int8 = std::int8_t;
using uint8 = std::uint8_t;
using int16 = std::int16_t;
using uint16 = std::uint16_t;
using int32 = std::int32_t;
using uint32 = std::uint32_t;
using int64 = std::int64_t;
using uint64 = std::uint64_t;

struct Entity;
using bodypair = Pair<Entity, Entity>;

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
constexpr decimal PI_RP3D = decimal(3.141592653589);

/// 2*Pi constant
constexpr decimal PI_TIMES_2 = decimal(6.28318530);

/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
/// inflated by a constant percentage of its size to allow the collision shape to move a little bit
/// without triggering a large modification of the tree each frame which can be costly
constexpr decimal DYNAMIC_TREE_FAT_AABB_INFLATE_PERCENTAGE = decimal(0.08);

/// Maximum number of contact points in a narrow phase info object
constexpr uint8 NB_MAX_CONTACT_POINTS_IN_NARROWPHASE_INFO = 16;

/// Maximum number of contact manifolds in an overlapping pair
constexpr uint8 NB_MAX_CONTACT_MANIFOLDS = 3;

/// Maximum number of potential contact manifolds in an overlapping pair
constexpr uint8 NB_MAX_POTENTIAL_CONTACT_MANIFOLDS = 4 * NB_MAX_CONTACT_MANIFOLDS;

/// Maximum number of contact points in potential contact manifold
constexpr uint16 NB_MAX_CONTACT_POINTS_IN_POTENTIAL_MANIFOLD = 256;

/// Distance threshold to consider that two contact points in a manifold are the same
constexpr decimal SAME_CONTACT_POINT_DISTANCE_THRESHOLD = decimal(0.01);

/// Current version of ReactPhysics3D
const std::string RP3D_VERSION = std::string("0.9.0");

}

#endif
