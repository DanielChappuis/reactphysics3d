/***************************************************************************
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

#ifndef CONTACTCACHE_H
#define	CONTACTCACHE_H

// Libraries
#include <map>
#include <vector>
#include <utility>
#include "ContactCachingInfo.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constant
const double POSITION_TOLERANCE = 0.01 ;      // Tolerance used to consider two similar positions to be considered as the same

/*  -------------------------------------------------------------------
    Class ContactCache :
       This class is used to cache the contact in order to be able to
       use the lambda values in the last time step in the next time step in the
       constraint solver to improve the convergence rate of the PGS algorithm.
       This class will cache the ContactCachingInfo objects at each time step.
    -------------------------------------------------------------------
*/
class ContactCache {
    private:
        std::map<std::pair<Body*, Body*>, std::vector<ContactCachingInfo> > cache;

    public:
        ContactCache();                                                                                         // Constructor
        ~ContactCache();                                                                                        // Destructor
        void clear();                                                                                           // Remove all the contact caching info of the cache
        void addContactCachingInfo(const ContactCachingInfo& contactCachingInfo);                               // Add a new contact caching info in the cache
        ContactCachingInfo* getContactCachingInfo(Body* body1, Body* body2, const Vector3D& position);    // Return the ContactCachingInfo (if exists) corresponding to the arguments
};

} // End of the ReactPhysics3D namespace

#endif

