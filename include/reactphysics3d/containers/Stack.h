/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <memory>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/memory/MemoryAllocator.h>

namespace reactphysics3d {

// Class Stack
/**
 * This class represents a simple generic stack.
  */
template<typename T>
class Stack {

    private:

        // -------------------- Attributes -------------------- //

        /// Reference to the memory allocator
        MemoryAllocator& mAllocator;

        /// Array that contains the elements of the stack
        T* mArray;

        /// Number of elements in the stack
        uint mNbElements;

        /// Number of allocated elements in the stack
        uint mCapacity;

        // -------------------- Methods -------------------- //

        /// Allocate more memory
        void allocate(size_t capacity) {

            T* newArray = static_cast<T*>(mAllocator.allocate(capacity * sizeof(T)));
            assert(newArray != nullptr);

            // If there
            if (mCapacity > 0) {

                if (mNbElements > 0) {

                    // Copy the previous items in the new array
                    std::uninitialized_copy(mArray, mArray + mNbElements, newArray);
                }

                // Release memory of the previous array
                mAllocator.release(mArray, mCapacity * sizeof(T));
            }

            mArray = newArray;

            mCapacity = capacity;
        }

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Stack(MemoryAllocator& allocator, size_t capacity = 0)
              :mAllocator(allocator), mArray(nullptr), mNbElements(0), mCapacity(0) {

            if (capacity > 0) {
                allocate(capacity);
            }
        }

        /// Copy constructor
        Stack(const Stack& stack)
              :mAllocator(stack.mAllocator), mArray(nullptr),
               mNbElements(stack.mNbElements), mCapacity(stack.mCapacity) {

            if (mCapacity > 0) {

                // Allocate memory for the buckets
                mArray = static_cast<T*>(mAllocator.allocate(mCapacity * sizeof(T)));
                assert(mArray != nullptr);

                if (mNbElements > 0) {

                    // Copy the items
                    std::uninitialized_copy(stack.mArray, stack.mArray + mNbElements, mArray);
                }
            }
        }

        /// Destructor
        ~Stack() {

            clear();

            if (mCapacity > 0) {

                // Release the memory allocated on the heap
                mAllocator.release(mArray, mCapacity * sizeof(T));
            }
        }

        /// Remove all the items from the stack
        void clear() {

            // Destruct the items
            for (size_t i = 0; i < mNbElements; i++) {
                mArray[i].~T();
            }

            mNbElements = 0;
        }

        /// Push an element into the stack
        void push(const T& element) {

            // If we need to allocate more elements
            if (mNbElements == mCapacity) {

                allocate(mCapacity > 0 ? mCapacity * 2 : 1);
            }

            // Copy the item into the array
            new (mArray + mNbElements) T(element);

            mNbElements++;
        }

        /// Pop an element from the stack (remove it from the stack and return it)
        T pop() {

            assert(mNbElements > 0);

            mNbElements--;

            // Copy the item
            T item = mArray[mNbElements];

            // Call the destructor
            mArray[mNbElements].~T();

            return item;
        }

        /// Return the number of items in the stack
        size_t size() const {
            return mNbElements;
        }

        /// Return the capacity of the stack
        size_t capacity() const {
            return mCapacity;
        }
};

}

#endif
