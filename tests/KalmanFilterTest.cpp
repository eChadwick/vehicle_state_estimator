#include <Eigen/Dense>
#include <string>

#include "KalmanFilter.hpp"
#include "ZeroAllocationTestFixture.hpp"
#include "gtest/gtest.h"

template <typename scalarType, int dimension>
using StateVector = Eigen::Matrix<scalarType, 1, dimension>;

template <typename scalarType, int dimension>
using Matrix = Eigen::Matrix<scalarType, dimension, dimension>;

template <typename T, int Dim>
struct KalmanFilterTestParams {
    StateVector<T, Dim> stateVector = StateVector<T, Dim>::Zero();
    Matrix<T, Dim> transitionMatrix = Matrix<T, Dim>::Identity();
    Matrix<T, Dim> errorCovarianceMatrix = Matrix<T, Dim>::Identity();
    Matrix<T, Dim> processNoiseMatrix = Matrix<T, Dim>::Identity();
};

template <typename T, int Dim>
KalmanFilter<T, Dim> KalmanFilterFromParams(KalmanFilterTestParams<T, Dim> params) {
    return KalmanFilter{params.stateVector, params.transitionMatrix, params.errorCovarianceMatrix,
                        params.processNoiseMatrix};
};

class KalmanFilterZeroAllocationTestFixture : public ZeroAllocationTestFixture {
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;
};

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedScalarTypes) {
    constexpr int dimension = 2;
    {
        KalmanFilterTestParams<double, dimension> params;
        KalmanFilter<double, dimension> uUt = KalmanFilterFromParams(params);

        EXPECT_TRUE(params.stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(params.transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(params.errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(params.processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }

    {
        KalmanFilterTestParams<float, dimension> params;
        KalmanFilter<float, dimension> uUt = KalmanFilterFromParams(params);

        EXPECT_TRUE(params.stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(params.transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(params.errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(params.processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }
}

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedVectorDimensions) {
    using ScalarType = double;

    {
        constexpr int dimension = 2;
        KalmanFilterTestParams<ScalarType, dimension> params;
        KalmanFilter<ScalarType, dimension> uUt = KalmanFilterFromParams(params);

        EXPECT_TRUE(params.stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(params.transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(params.errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(params.processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }

    {
        constexpr int dimension = 3;
        KalmanFilterTestParams<ScalarType, dimension> params;
        KalmanFilter<ScalarType, dimension> uUt = KalmanFilterFromParams(params);

        EXPECT_TRUE(params.stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(params.transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(params.errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(params.processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }
}

TEST_F(KalmanFilterZeroAllocationTestFixture, InitWithAsymmetricErrorCovarianceIsError) {
    dynamic_allocation_allowed = true;
    KalmanFilterTestParams<double, 2> params;
    params.errorCovarianceMatrix(1, 0)++;
    EXPECT_THROW(KalmanFilterFromParams(params), std::invalid_argument);
}

TEST_F(KalmanFilterZeroAllocationTestFixture, InitWithAsymmetricProcessNoiseIsError) {
    dynamic_allocation_allowed = true;
    KalmanFilterTestParams<double, 2> params;
    params.processNoiseMatrix(1, 0)++;
    EXPECT_THROW(KalmanFilterFromParams(params), std::invalid_argument);
}

TEST_F(KalmanFilterZeroAllocationTestFixture, InitWithInvalidNumericInputsIsError) {
    dynamic_allocation_allowed = true;

    const double invalidValues[] = {std::numeric_limits<double>::quiet_NaN(),
                                    std::numeric_limits<double>::infinity()};

    for (double badVal : invalidValues) {
        {
            KalmanFilterTestParams<double, 2> params;
            params.stateVector(0) = badVal;
            EXPECT_THROW(KalmanFilterFromParams(params), std::invalid_argument);
        }

        {
            KalmanFilterTestParams<double, 2> params;
            params.transitionMatrix(0, 0) = badVal;
            EXPECT_THROW(KalmanFilterFromParams(params), std::invalid_argument);
        }

        {
            KalmanFilterTestParams<double, 2> params;
            params.errorCovarianceMatrix(0, 0) = badVal;
            EXPECT_THROW(KalmanFilterFromParams(params), std::invalid_argument);
        }

        {
            KalmanFilterTestParams<double, 2> params;
            params.processNoiseMatrix(0, 0) = badVal;
            EXPECT_THROW(KalmanFilterFromParams(params), std::invalid_argument);
        }
    }
}