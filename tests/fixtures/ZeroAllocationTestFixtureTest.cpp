#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <memory>
#include <new>

#include "ZeroAllocationTestFixture.hpp"

class ConcreteZeroAllocationTestFixture : public ZeroAllocationTestFixture {
  public:
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;

    void TestBody() override {}
};

TEST(ZeroAllocationTestFixtureTest, ScalarOperatorNewTriggersFailure) {
    EXPECT_NONFATAL_FAILURE(
        {
            ConcreteZeroAllocationTestFixture fixture;
            fixture.SetUp();
            ::operator delete(::operator new(32));
            fixture.TearDown();
        },
        allocation_failure_message);
}

TEST(ZeroAllocationTestFixtureTest, NothrowScalarOperatorNewTriggersFailure) {
    EXPECT_NONFATAL_FAILURE(
        {
            ConcreteZeroAllocationTestFixture fixture;
            fixture.SetUp();
            ::operator delete(::operator new(32, std::nothrow));
            fixture.TearDown();
        },
        allocation_failure_message);
}

TEST(ZeroAllocationTestFixtureTest, ArrayOperatorNewTriggersFailure) {
    EXPECT_NONFATAL_FAILURE(
        {
            ConcreteZeroAllocationTestFixture fixture;
            fixture.SetUp();
            ::operator delete[](::operator new[](32));
            fixture.TearDown();
        },
        allocation_failure_message);
}

TEST(ZeroAllocationTestFixtureTest, NothrowArrayOperatorNewTriggersFailure) {
    EXPECT_NONFATAL_FAILURE(
        {
            ConcreteZeroAllocationTestFixture fixture;
            fixture.SetUp();
            ::operator delete[](::operator new[](32, std::nothrow));
            fixture.TearDown();
        },
        allocation_failure_message);
}

TEST(ZeroAllocationTestFixtureTest, AllocationCanBeExplicitlyAllowed) {
    dynamic_allocation_allowed = true;
    ConcreteZeroAllocationTestFixture fixture;
    fixture.SetUp();
    ::operator delete(::operator new(32));
    ::operator delete(::operator new(32, std::nothrow));
    ::operator delete[](::operator new[](32));
    ::operator delete[](::operator new[](32, std::nothrow));
    fixture.TearDown();
}

TEST(ZeroAllocationTestFixtureTest, NewTestFixtureDisallowsDynamicAllocation) {
    dynamic_allocation_allowed = true;
    ConcreteZeroAllocationTestFixture fixture;
    fixture.SetUp();
    ::operator delete(::operator new(32));
    fixture.TearDown();

    dynamic_allocation_allowed = false;
    EXPECT_NONFATAL_FAILURE(
        {
            ConcreteZeroAllocationTestFixture newFixture;
            newFixture.SetUp();
            ::operator delete(::operator new(32));
            newFixture.TearDown();
        },
        allocation_failure_message);
}