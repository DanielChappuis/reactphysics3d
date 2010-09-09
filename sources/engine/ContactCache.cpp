/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

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
void ContactCache::addContactCachingInfo(ContactCachingInfo* info) {
    // Check if there is already an entry for that pair of body in the cache
    map<pair<Body*, Body*>, vector<ContactCachingInfo*> >::iterator entry = cache.find(make_pair(info->body1, info->body2));
    if (entry != cache.end()) {
        (*entry).second.push_back(info);
    }
    else {
        // Add a new entry in the cache
        vector<ContactCachingInfo*> vec;
        vec.push_back(info);
        cache.insert(make_pair(make_pair(info->body1, info->body2), vec));
    }
}

// Return the ContactCachingInfo (if exists) corresponding to the arguments
// If the contact pair between the two bodies in argument doesn't exist this or there is no ContactCachingInfo
// compatible (with approximatively the same position), this method returns NULL.
ContactCachingInfo* ContactCache::getContactCachingInfo(Body* body1, Body* body2, const Vector3D& position) {
   // Check if there is an entry for that pair of body in the cache
    map<pair<Body*, Body*>, vector<ContactCachingInfo*> >::iterator entry = cache.find(make_pair(body1, body2));
    if (entry != cache.end()) {
        vector<ContactCachingInfo*> vec = (*entry).second;
        assert((*entry).first.first == body1);
        assert((*entry).first.second == body2);

        double posX = position.getX();
        double posY = position.getY();
        double posZ = position.getZ();

        // Check among all the old contacts for this pair of body if there is an approximative match for the contact position
        for(vector<ContactCachingInfo*>::iterator it = vec.begin(); it != vec.end(); it++) {
            Vector3D& contactPos = (*it)->position;
            if (posX <= contactPos.getX() + POSITION_TOLERANCE && posX >= contactPos.getX() - POSITION_TOLERANCE &&
                posY <= contactPos.getY() + POSITION_TOLERANCE && posY >= contactPos.getY() - POSITION_TOLERANCE &&
                posZ <= contactPos.getZ() + POSITION_TOLERANCE && posZ >= contactPos.getZ() - POSITION_TOLERANCE) {
                // Return the ContactCachingInfo
                return *it;
            }
        }
    }
    else {
        return 0;
    }
}

