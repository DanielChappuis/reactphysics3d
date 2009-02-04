/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by     *
 * the Free Software Foundation, either version 3 of the License, or        *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU General Public License for more details.                             *
 *                                                                          *
 * You should have received a copy of the GNU General Public License        *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

#ifndef CONTEXT_H
#define CONTEXT_H

// Libraries
#include "Objects.h"

#include <vector>

// Class Context
class Context {
    private :
        std::vector<Object*> vectObjects;               // Vector of Objects in the simulation

    public :
        Context();                                      // Constructor of the class
        ~Context();                                     // Destructor of the class
        int getNbObjects() const;                       // Return the number of objects in the context
        Object& getObject(int objectIndex) const;       // Get an object from the context
        void addObject(Object* object);                 // Add an object into the context
        void removeObject(int objectIndex);             // Remove an object from the context
};

// Method (inline) to get the number of objects in the context
inline int Context::getNbObjects() const {
    return vectObjects.size();
}

#endif
