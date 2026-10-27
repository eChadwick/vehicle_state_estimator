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
        using stateVector = Eigen::Matrix<double, 1, 2>;
        using matrix = Eigen::Matrix<double, 2, 2>;

        stateVector stateVector_double;
        matrix transitionMatrix_double;
        matrix errorCovarianceMatrix_double;
        matrix processNoiseMatrix_double;
        KalmanFilter uUt_double{stateVector_double, transitionMatrix_double,
                                errorCovarianceMatrix_double, processNoiseMatrix_double};

        EXPECT_TRUE(stateVector_double.isApprox(uUt_double.GetStateVector()));
        EXPECT_TRUE(transitionMatrix_double.isApprox(uUt_double.GetTransitionMatrix()));
        EXPECT_TRUE(errorCovarianceMatrix_double.isApprox(uUt_double.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(processNoiseMatrix_double.isApprox(uUt_double.GetProcessNoiseMatrix()));
    }

    {
        using stateVector = Eigen::Matrix<float, 1, 2>;
        using matrix = Eigen::Matrix<float, 2, 2>;

        stateVector stateVector_float;
        matrix transitionMatrix_float;
        matrix errorCovarianceMatrix_float;
        matrix processNoiseMatrix_float;
        KalmanFilter uUt_float{stateVector_float, transitionMatrix_float,
                               errorCovarianceMatrix_float, processNoiseMatrix_float};

        EXPECT_TRUE(stateVector_float.isApprox(uUt_float.GetStateVector()));
        EXPECT_TRUE(transitionMatrix_float.isApprox(uUt_float.GetTransitionMatrix()));
        EXPECT_TRUE(errorCovarianceMatrix_float.isApprox(uUt_float.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(processNoiseMatrix_float.isApprox(uUt_float.GetProcessNoiseMatrix()));
    }
}