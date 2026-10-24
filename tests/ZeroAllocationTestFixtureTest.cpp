#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

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