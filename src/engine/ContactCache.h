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

#ifndef CONTACT_CACHE_H
#define	CONTACT_CACHE_H

// Libraries
#include <map>
#include <vector>
#include <utility>
#include "ContactCachingInfo.h"
#include "../constraint/Contact.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constant
const double POSITION_TOLERANCE = 0.01;      // Tolerance used to consider two similar positions to be considered as the same

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
        std::map<std::pair<Body*, Body*>, ContactCachingInfo* > cache;

    public:
        ContactCache();                                                         // Constructor
        ~ContactCache();                                                        // Destructor
        void clear();                                                           // Remove all the contact caching info of the cache
        ContactCachingInfo* getContactCachingInfo(Contact* contact) const;      // Return the ContactCachingInfo corresponding to a contact if it exists
        void addContactCachingInfo(ContactCachingInfo* contactCachingInfo);     // Add a new contact caching info in the cache
       
};

} // End of the ReactPhysics3D namespace

#endif

