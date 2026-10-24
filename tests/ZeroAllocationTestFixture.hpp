#include <gtest/gtest.h>

inline thread_local bool allocation_failure{false};
inline const char* allocation_failure_message{"Heap allocation failure"};

void* operator new(size_t size);
void* operator new(size_t size, const std::nothrow_t&) noexcept;
void* operator new[](size_t size);
void* operator new[](size_t size, const std::nothrow_t&) noexcept;

void operator delete(void* ptr) noexcept;
void operator delete[](void* ptr) noexcept;
void operator delete(void* ptr, size_t) noexcept;
void operator delete[](void* ptr, size_t) noexcept;

class ZeroAllocationTestFixture : public testing::Test {
  protected:
    void SetUp() override { allocation_failure = false; }

    void TearDown() override { EXPECT_FALSE(allocation_failure) << allocation_failure_message; }
};
