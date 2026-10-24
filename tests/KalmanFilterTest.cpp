#include <gtest/gtest.h>

#include "KalmanFilter.hpp"
#include "ZeroAllocationTestFixture.hpp"

class KalmanFilterZeroAllocationTestFixture : public ZeroAllocationTestFixture {
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;
};

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedConstruction) {
    KalmanFilter();
}