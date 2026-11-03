#include "ZeroAllocationTestFixture.hpp"

#include <cstdlib>

void* operator new(size_t size) {
    if (!dynamic_allocation_allowed) {
        allocation_failure = true;
    }
    return std::malloc(size);
}

void* operator new(size_t size, const std::nothrow_t&) noexcept {
    if (!dynamic_allocation_allowed) {
        allocation_failure = true;
    }
    return std::malloc(size);
}

void* operator new[](size_t size) {
    if (!dynamic_allocation_allowed) {
        allocation_failure = true;
    }
    return std::malloc(size);
}

void* operator new[](size_t size, const std::nothrow_t&) noexcept {
    if (!dynamic_allocation_allowed) {
        allocation_failure = true;
    }
    return std::malloc(size);
}

void operator delete(void* ptr) noexcept {
    std::free(ptr);
}
void operator delete[](void* ptr) noexcept {
    std::free(ptr);
}
void operator delete(void* ptr, size_t) noexcept {
    std::free(ptr);
}
void operator delete[](void* ptr, size_t) noexcept {
    std::free(ptr);
}