#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>

#include "ZeroAllocationTestFixture.hpp"

class ConcreteZeroAllocationTestFixture : public ZeroAllocationTestFixture{
    public:
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;

    void TestBody() override {}
};

TEST(ZeroAllocationTestFixtureTest, DynamicAllocationCausesTestFailure){
    EXPECT_NONFATAL_FAILURE({
        ConcreteZeroAllocationTestFixture fixture;        
        fixture.SetUp();
        
        void* ptr = ::operator new(32);
        ::operator delete(ptr);
        
        fixture.TearDown();
        
    }, allocation_failure_message);
}