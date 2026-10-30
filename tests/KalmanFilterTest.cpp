#include <Eigen/Dense>
#include <string>

#include "KalmanFilter.hpp"
#include "ZeroAllocationTestFixture.hpp"
#include "gtest/gtest.h"

class KalmanFilterZeroAllocationTestFixture : public ZeroAllocationTestFixture {
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;
};

template <typename T, int Dim>
void VerifyKalmanFilterPassthrough() {
    using StateVector = Eigen::Matrix<T, 1, Dim>;
    using Matrix = Eigen::Matrix<T, Dim, Dim>;

    StateVector stateVector;
    Matrix transitionMatrix;
    Matrix errorCovarianceMatrix;
    Matrix processNoiseMatrix;

    KalmanFilter<T, Dim> uUt{stateVector, transitionMatrix, errorCovarianceMatrix,
                             processNoiseMatrix};

    EXPECT_TRUE(stateVector.isApprox(uUt.GetStateVector()));
    EXPECT_TRUE(transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
    EXPECT_TRUE(errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
    EXPECT_TRUE(processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
}

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedScalarTypes) {
    VerifyKalmanFilterPassthrough<double, 2>();
    VerifyKalmanFilterPassthrough<float, 2>();
}

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedVectorDimensions) {
    VerifyKalmanFilterPassthrough<float, 2>();
    VerifyKalmanFilterPassthrough<float, 3>();
}