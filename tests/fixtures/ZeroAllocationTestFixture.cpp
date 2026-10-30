#include "ZeroAllocationTestFixture.hpp"

#include <cstdlib>

void* operator new(size_t size) {
    allocation_failure = true;
    return std::malloc(size);
}

void* operator new(size_t size, const std::nothrow_t&) noexcept {
    allocation_failure = true;
    return std::malloc(size);
}

void* operator new[](size_t size) {
    allocation_failure = true;
    return std::malloc(size);
}

void* operator new[](size_t size, const std::nothrow_t&) noexcept {
    allocation_failure = true;
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