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

// Libraries
#include "ContactCache.h"

using namespace reactphysics3d;
using namespace std;

// Constructor
ContactCache::ContactCache() {

}

// Destructor
ContactCache::~ContactCache() {
    
}

// Remove all the contact caching info of the cache
void ContactCache::clear() {
    // Clear the cache
    cache.clear();
}

// Add a new contact caching info in the cache
void ContactCache::addContactCachingInfo(const ContactCachingInfo& info) {
    // Check if there is already an entry for that pair of body in the cache
    map<pair<Body*, Body*>, vector<ContactCachingInfo> >::iterator entry = cache.find(make_pair(info.body1, info.body2));
    if (entry != cache.end()) {
        (*entry).second.push_back(info);
    }
    else {
        // Add a new entry in the cache
        vector<ContactCachingInfo> vec;
        vec.push_back(info);
        cache.insert(make_pair(make_pair(info.body1, info.body2), vec));
    }
}

// Return the ContactCachingInfo (if exists) corresponding to the arguments
// If the contact pair between the two bodies in argument doesn't exist this or there is no ContactCachingInfo
// compatible (with approximatively the same position), this method returns NULL.
ContactCachingInfo* ContactCache::getContactCachingInfo(Body* body1, Body* body2, const Vector3D& position) {
   // Check if there is an entry for that pair of body in the cache
    map<pair<Body*, Body*>, vector<ContactCachingInfo> >::iterator entry = cache.find(make_pair(body1, body2));
    if (entry != cache.end()) {
        vector<ContactCachingInfo> vec = (*entry).second;
        assert((*entry).first.first == body1);
        assert((*entry).first.second == body2);

        double posX = position.getX();
        double posY = position.getY();
        double posZ = position.getZ();

        // Check among all the old contacts for this pair of body if there is an approximative match for the contact position
        for(vector<ContactCachingInfo>::iterator it = vec.begin(); it != vec.end(); it++) {
            Vector3D& contactPos = (*it).position;
            if (posX <= contactPos.getX() + POSITION_TOLERANCE && posX >= contactPos.getX() - POSITION_TOLERANCE &&
                posY <= contactPos.getY() + POSITION_TOLERANCE && posY >= contactPos.getY() - POSITION_TOLERANCE &&
                posZ <= contactPos.getZ() + POSITION_TOLERANCE && posZ >= contactPos.getZ() - POSITION_TOLERANCE) {
                // Return the ContactCachingInfo
                return &(*it);
            }
        }
    }
    else {
        return 0;
    }
}

