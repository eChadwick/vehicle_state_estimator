#include <gtest/gtest.h>

#include <Eigen/Dense>  // IWYU pragma: keep

#include "KalmanFilter.hpp"
#include "ZeroAllocationTestFixture.hpp"

class KalmanFilterZeroAllocationTestFixture : public ZeroAllocationTestFixture {
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;
};

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedScalarTypes) {
    using stateVector = Eigen::Matrix<double, 1, 2>;
    using matrix = Eigen::Matrix<double, 2, 2>;

    stateVector stateVector_double;
    matrix transitionMatrix_double;
    matrix errorCovarianceMatrix_double;
    matrix processNoiseMatrix_double;
    KalmanFilter uUt{stateVector_double, transitionMatrix_double, errorCovarianceMatrix_double,
                     processNoiseMatrix_double};

    EXPECT_TRUE(stateVector_double.isApprox(uUt.GetStateVector()));
    EXPECT_TRUE(transitionMatrix_double.isApprox(uUt.GetTransitionMatrix()));
    EXPECT_TRUE(errorCovarianceMatrix_double.isApprox(uUt.GetErrorCovarianceMatrix()));
    EXPECT_TRUE(processNoiseMatrix_double.isApprox(uUt.GetProcessNoiseMatrix()));

    // Eigen::Matrix<float, 1, 2> stateVector;
    // Eigen::Matrix<float, 2, 2> transitionMatrix;
    // Eigen::Matrix<float, 2, 2> errorCovarianceMatrix;
    // Eigen::Matrix<float, 2, 2> processNoiseMatrix;
    // KalmanFilter uUt{stateVector, transitionMatrix, errorCovarianceMatrix, processNoiseMatrix};

    // EXPECT_TRUE(stateVector.isApprox(uUt.GetStateVector()));
    // EXPECT_TRUE(transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
    // EXPECT_TRUE(errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
    // EXPECT_TRUE(processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
}