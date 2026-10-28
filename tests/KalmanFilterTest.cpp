#include <gtest/gtest.h>

#include <Eigen/Dense>  // IWYU pragma: keep

#include "KalmanFilter.hpp"
#include "ZeroAllocationTestFixture.hpp"

class KalmanFilterZeroAllocationTestFixture : public ZeroAllocationTestFixture {
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;
};

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedScalarTypes) {
    {
        using StateVector = Eigen::Matrix<double, 1, 2>;
        using Matrix = Eigen::Matrix<double, 2, 2>;

        StateVector stateVector;
        Matrix transitionMatrix;
        Matrix errorCovarianceMatrix;
        Matrix processNoiseMatrix;
        KalmanFilter uUt{stateVector, transitionMatrix, errorCovarianceMatrix, processNoiseMatrix};

        EXPECT_TRUE(stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }

    {
        using StateVector = Eigen::Matrix<float, 1, 2>;
        using Matrix = Eigen::Matrix<float, 2, 2>;

        StateVector stateVector;
        Matrix transitionMatrix;
        Matrix errorCovarianceMatrix;
        Matrix processNoiseMatrix;
        KalmanFilter uUt{stateVector, transitionMatrix, errorCovarianceMatrix, processNoiseMatrix};

        EXPECT_TRUE(stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }
}

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedVectorDimensions) {
    {
        using StateVector = Eigen::Matrix<float, 1, 2>;
        using Matrix = Eigen::Matrix<float, 2, 2>;

        StateVector stateVector;
        Matrix transitionMatrix;
        Matrix errorCovarianceMatrix;
        Matrix processNoiseMatrix;
        KalmanFilter uUt{stateVector, transitionMatrix, errorCovarianceMatrix, processNoiseMatrix};

        EXPECT_TRUE(stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }

    {
        using StateVector = Eigen::Matrix<float, 1, 3>;
        using Matrix = Eigen::Matrix<float, 3, 3>;

        StateVector stateVector;
        Matrix transitionMatrix;
        Matrix errorCovarianceMatrix;
        Matrix processNoiseMatrix;
        KalmanFilter uUt{stateVector, transitionMatrix, errorCovarianceMatrix, processNoiseMatrix};

        EXPECT_TRUE(stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }
}