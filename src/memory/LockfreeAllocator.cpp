#include <reactphysics3d/memory/MemoryManager.h>
#include <reactphysics3d/memory/LockfreeAllocator.h>

using namespace reactphysics3d;

typedef struct MemHead {
  /* Length of allocated memory block. */
  size_t len;
} MemHead;

#define MEMHEAD_FROM_PTR(ptr) (((MemHead *)ptr) - 1)
#define PTR_FROM_MEMHEAD(memhead) (memhead + 1)
#define MEMHEAD_LEN(memhead) ((memhead)->len)

void *LockfreeAllocator::allocate ( size_t size ) {
    MemHead *memhead = (MemHead *)malloc(size + sizeof(MemHead));
    if (memhead) {
        memhead->len = size;
        return PTR_FROM_MEMHEAD(memhead);
    }
    return NULL;
}

void *LockfreeAllocator::duplicate(const void *vmem) {
    const size_t len = getAllocLen(vmem);
    void *ptr = allocate(len);
        if (ptr) {
        memcpy(ptr, vmem, len);
    }
    return ptr;
}

size_t LockfreeAllocator::getAllocLen ( const void *vmem ) {
    if (vmem) {
        return MEMHEAD_LEN(MEMHEAD_FROM_PTR(vmem));
    }
    return 0;
}

void LockfreeAllocator::release ( void *pointer , size_t /*size*/ ) {
    free(MEMHEAD_FROM_PTR(pointer));
}
