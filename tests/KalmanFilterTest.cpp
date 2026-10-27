#include <gtest/gtest.h>

#include <Eigen/Dense>  // IWYU pragma: keep

#include "KalmanFilter.hpp"
#include "ZeroAllocationTestFixture.hpp"

class KalmanFilterZeroAllocationTestFixture : public ZeroAllocationTestFixture {
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;
};

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedConstruction) {
    Eigen::Matrix<double, 1, 2> stateVector;
    Eigen::Matrix<double, 2, 2> transitionMatrix;
    Eigen::Matrix<double, 2, 2> errorCovarianceMatrix;
    Eigen::Matrix<double, 2, 2> processNoiseMatrix;
    KalmanFilter uUt{stateVector, transitionMatrix, errorCovarianceMatrix, processNoiseMatrix};

    EXPECT_TRUE(stateVector.isApprox(uUt.GetStateVector()));
    EXPECT_TRUE(transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
    EXPECT_TRUE(errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
    EXPECT_TRUE(processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
}