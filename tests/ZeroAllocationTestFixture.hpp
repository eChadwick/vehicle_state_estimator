#include <gtest/gtest.h>

inline thread_local bool allocation_failure{false};
const char* allocation_failure_message{"Heap allocation failure"};

void* operator new(size_t size) {
    allocation_failure = true;
    return std::malloc(size);
}

void* operator new(size_t size, const std::nothrow_t&) noexcept {
    allocation_failure = true;
    return std::malloc(size);
}

void* operator new[](size_t size) {
    // allocation_failure = true;
    return std::malloc(size);
}

void* operator new[](size_t size, const std::nothrow_t&) noexcept {
    // allocation_failure = true;
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

class ZeroAllocationTestFixture : public testing::Test {
  protected:
    void SetUp() override { allocation_failure = false; }

    void TearDown() override { EXPECT_FALSE(allocation_failure) << allocation_failure_message; }
};
