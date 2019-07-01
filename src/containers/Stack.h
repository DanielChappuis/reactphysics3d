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

#ifndef REACTPHYSICS3D_STACK_H
#define REACTPHYSICS3D_STACK_H

// Libraries
#include "configuration.h"
#include "memory/MemoryAllocator.h"

namespace reactphysics3d {

// Class Stack
/**
 * This class represents a simple generic stack with an initial capacity. If the number
 * of elements exceeds the capacity, the heap will be used to allocated more memory.
  */
template<typename T, uint capacity>
class Stack {

    private:

        // -------------------- Attributes -------------------- //

        /// Reference to the memory allocator
        MemoryAllocator& mAllocator;

        /// Initial array that contains the elements of the stack
        T mInitArray[capacity];

        /// Pointer to the first element of the stack
        T* mElements;

        /// Number of elements in the stack
        uint mNbElements;

        /// Number of allocated elements in the stack
        uint mNbAllocatedElements;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Stack(MemoryAllocator& allocator)
              :mAllocator(allocator), mElements(mInitArray), mNbElements(0), mNbAllocatedElements(capacity) {

        }

        /// Destructor
        ~Stack() {

            // If elements have been allocated on the heap
            if (mInitArray != mElements) {

                // Release the memory allocated on the heap
                mAllocator.release(mElements, mNbAllocatedElements * sizeof(T));
            }
        }

        /// Push an element into the stack
        void push(const T& element);

        /// Pop an element from the stack (remove it from the stack and return it)
        T pop();

        /// Return the number of elments in the stack
        uint getNbElements() const;

};

// Push an element into the stack
template<typename T, uint capacity>
inline void Stack<T, capacity>::push(const T& element) {

    // If we need to allocate more elements
    if (mNbElements == mNbAllocatedElements) {
        T* oldElements = mElements;
        uint oldNbAllocatedElements = mNbAllocatedElements;
        mNbAllocatedElements *= 2;
        mElements = static_cast<T*>(mAllocator.allocate(mNbAllocatedElements * sizeof(T)));
        assert(mElements);
        memcpy(mElements, oldElements, mNbElements * sizeof(T));
        if (oldElements != mInitArray) {
            mAllocator.release(oldElements, oldNbAllocatedElements * sizeof(T));
        }
    }

    mElements[mNbElements] = element;
    mNbElements++;
}

// Pop an element from the stack (remove it from the stack and return it)
template<typename T, uint capacity>
inline T Stack<T, capacity>::pop() {
    assert(mNbElements > 0);
    mNbElements--;
    return mElements[mNbElements];
}

// Return the number of elments in the stack
template<typename T, uint capacity>
inline uint Stack<T, capacity>::getNbElements() const {
    return mNbElements;
}

}

#endif
