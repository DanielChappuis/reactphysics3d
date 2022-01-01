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

#ifndef REACTPHYSICS3D_LINKED_LIST_H
#define REACTPHYSICS3D_LINKED_LIST_H

// Libraries
#include <reactphysics3d/memory/MemoryAllocator.h>

namespace reactphysics3d {

// Class LinkedList
/**
 * This class represents a simple generic linked list.
  */
template<typename T>
class LinkedList {

    public:

        /// Element of the linked list
        struct ListElement {

            /// Data of the list element
            T data;

            /// Pointer to the next element of the list
            ListElement* next;

            /// Constructor
            ListElement(T elementData, ListElement* nextElement)
                : data(elementData), next(nextElement) {

            }
        };

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the first element of the list
        ListElement* mListHead;

        /// Memory allocator used to allocate the list elements
        MemoryAllocator& mAllocator;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        LinkedList(MemoryAllocator& allocator) : mListHead(nullptr), mAllocator(allocator) {

        }

        /// Destructor
        ~LinkedList() {
            reset();
        }

        /// Return the first element of the list
        ListElement* getListHead() const;

        /// Insert an element at the beginning of the linked list
        void insert(const T& data);

        /// Remove all the elements of the list
        void reset();

};

// Return the first element of the list
template<typename T>
RP3D_FORCE_INLINE typename LinkedList<T>::ListElement* LinkedList<T>::getListHead() const {
    return mListHead;
}

// Insert an element at the beginning of the linked list
template<typename T>
RP3D_FORCE_INLINE void LinkedList<T>::insert(const T& data) {
    ListElement* element = new (mAllocator.allocate(sizeof(ListElement))) ListElement(data, mListHead);
    mListHead = element;
}

// Remove all the elements of the list
template<typename T>
RP3D_FORCE_INLINE void LinkedList<T>::reset() {

    // Release all the list elements
    ListElement* element = mListHead;
    while (element != nullptr) {
        ListElement* nextElement = element->next;
        mAllocator.release(element, sizeof(ListElement));
        element = nextElement;
    }

    mListHead = nullptr;
}

}

#endif
