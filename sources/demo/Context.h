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
