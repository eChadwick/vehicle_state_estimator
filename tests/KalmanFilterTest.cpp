#include <Eigen/Dense>
#include <string>

#include "KalmanFilter.hpp"
#include "ZeroAllocationTestFixture.hpp"
#include "gtest/gtest.h"

template <typename ScalarType, int dimension>
using StateVector = Eigen::Matrix<ScalarType, dimension, 1>;

template <typename ScalarType, int dimension>
using Matrix = Eigen::Matrix<ScalarType, dimension, dimension>;

template <typename ScalarType, int StateDimension>
struct KalmanFilterTestParams {
    StateVector<ScalarType, StateDimension> stateVector =
        StateVector<ScalarType, StateDimension>::Zero();
    Matrix<ScalarType, StateDimension> transitionMatrix =
        Matrix<ScalarType, StateDimension>::Identity();
    Matrix<ScalarType, StateDimension> errorCovarianceMatrix =
        Matrix<ScalarType, StateDimension>::Identity();
    Matrix<ScalarType, StateDimension> processNoiseMatrix =
        Matrix<ScalarType, StateDimension>::Identity();
};

template <typename ScalarType, int StateDimension>
KalmanFilter<ScalarType, StateDimension> KalmanFilterFromParams(
    KalmanFilterTestParams<ScalarType, StateDimension> params) {
    return KalmanFilter{params.stateVector, params.transitionMatrix, params.errorCovarianceMatrix,
                        params.processNoiseMatrix};
};

class KalmanFilterZeroAllocationTestFixture : public ZeroAllocationTestFixture {
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;
};

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedScalarTypes) {
    constexpr int StateDimension = 2;
    {
        KalmanFilterTestParams<double, StateDimension> params;
        KalmanFilter<double, StateDimension> uUt = KalmanFilterFromParams(params);

        EXPECT_TRUE(params.stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(params.transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(params.errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(params.processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }

    {
        KalmanFilterTestParams<float, StateDimension> params;
        KalmanFilter<float, StateDimension> uUt = KalmanFilterFromParams(params);

        EXPECT_TRUE(params.stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(params.transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(params.errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(params.processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }
}

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedStateDimensions) {
    using ScalarType = double;

    {
        constexpr int StateDimension = 2;
        KalmanFilterTestParams<ScalarType, StateDimension> params;
        KalmanFilter<ScalarType, StateDimension> uUt = KalmanFilterFromParams(params);

        EXPECT_TRUE(params.stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(params.transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(params.errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(params.processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }

    {
        constexpr int StateDimension = 3;
        KalmanFilterTestParams<ScalarType, StateDimension> params;
        KalmanFilter<ScalarType, StateDimension> uUt = KalmanFilterFromParams(params);

        EXPECT_TRUE(params.stateVector.isApprox(uUt.GetStateVector()));
        EXPECT_TRUE(params.transitionMatrix.isApprox(uUt.GetTransitionMatrix()));
        EXPECT_TRUE(params.errorCovarianceMatrix.isApprox(uUt.GetErrorCovarianceMatrix()));
        EXPECT_TRUE(params.processNoiseMatrix.isApprox(uUt.GetProcessNoiseMatrix()));
    }
}

TEST_F(KalmanFilterZeroAllocationTestFixture, PredictStep) {
    using ScalarType = double;
    constexpr int StateDimension = 2;

    KalmanFilterTestParams<ScalarType, StateDimension> params;
    params.stateVector << 1, 2;
    // clang-format off
    params.transitionMatrix << 1, 2, 
                               3, 4;
    params.errorCovarianceMatrix << 2, 1,
                                    1, 3;
    params.processNoiseMatrix << 0.5, 0.1,
                                 0.1, 0.25;
    // clang-format on
    KalmanFilter<ScalarType, StateDimension> uUt = KalmanFilterFromParams(params);

    uUt.Predict();

    StateVector<ScalarType, StateDimension> expectedStateVector;
    expectedStateVector << 5, 11;
    Matrix<ScalarType, StateDimension> expectedErrorCovarianceMatrix;
    // clang-format off
    expectedErrorCovarianceMatrix << 18.5, 40.1,
                                     40.1, 90.25;
    // clang-format on
    EXPECT_TRUE(uUt.GetStateVector().isApprox(expectedStateVector));
    EXPECT_TRUE(uUt.GetErrorCovarianceMatrix().isApprox(expectedErrorCovarianceMatrix));
}

TEST(KalmanFilterInitTests, InitWithAsymmetricErrorCovarianceIsError) {
    KalmanFilterTestParams<double, 2> params;
    params.errorCovarianceMatrix(1, 0)++;

    EXPECT_THROW(
        {
            try {
                KalmanFilterFromParams(params);
            } catch (const std::invalid_argument& e) {
                EXPECT_STREQ(e.what(), KalmanFilterErrors::AsymmetricErrorCovariance);
                throw;
            }
        },
        std::invalid_argument);
}

TEST(KalmanFilterInitTests, InitWithAsymmetricProcessNoiseIsError) {
    KalmanFilterTestParams<double, 2> params;
    params.processNoiseMatrix(1, 0)++;

    EXPECT_THROW(
        {
            try {
                KalmanFilterFromParams(params);
            } catch (const std::invalid_argument& e) {
                EXPECT_STREQ(e.what(), KalmanFilterErrors::AsymmetricProcessNoise);
                throw;
            }
        },
        std::invalid_argument);
}

TEST(KalmanFilterInitTests, InitWithInvalidNumericInputsIsError) {
    const double invalidValues[] = {std::numeric_limits<double>::quiet_NaN(),
                                    std::numeric_limits<double>::infinity()};

    for (double badVal : invalidValues) {
        {
            KalmanFilterTestParams<double, 2> params;
            params.stateVector(0) = badVal;

            EXPECT_THROW(
                {
                    try {
                        KalmanFilterFromParams(params);
                    } catch (const std::invalid_argument& e) {
                        EXPECT_STREQ(e.what(), KalmanFilterErrors::NonFiniteStateVector);
                        throw;
                    }
                },
                std::invalid_argument);
        }

        {
            KalmanFilterTestParams<double, 2> params;
            params.transitionMatrix(0, 0) = badVal;

            EXPECT_THROW(
                {
                    try {
                        KalmanFilterFromParams(params);
                    } catch (const std::invalid_argument& e) {
                        EXPECT_STREQ(e.what(), KalmanFilterErrors::NonFiniteTransitionMatrix);
                        throw;
                    }
                },
                std::invalid_argument);
        }

        {
            KalmanFilterTestParams<double, 2> params;
            params.errorCovarianceMatrix(0, 0) = badVal;

            EXPECT_THROW(
                {
                    try {
                        KalmanFilterFromParams(params);
                    } catch (const std::invalid_argument& e) {
                        EXPECT_STREQ(e.what(), KalmanFilterErrors::NonFiniteErrorCovarianceMatrix);
                        throw;
                    }
                },
                std::invalid_argument);
        }

        {
            KalmanFilterTestParams<double, 2> params;
            params.processNoiseMatrix(0, 0) = badVal;

            EXPECT_THROW(
                {
                    try {
                        KalmanFilterFromParams(params);
                    } catch (const std::invalid_argument& e) {
                        EXPECT_STREQ(e.what(), KalmanFilterErrors::NonFiniteProcessNoiseMatrix);
                        throw;
                    }
                },
                std::invalid_argument);
        }
    }
}

TEST(KalmanFilterInitTests, InitWithNonPositiveSemiDefiniteErrorCovarianceMatrixIsError) {
    {
        KalmanFilterTestParams<double, 2> params;
        params.errorCovarianceMatrix(1, 1) *= -1;

        EXPECT_THROW(
            {
                try {
                    KalmanFilterFromParams(params);
                } catch (const std::invalid_argument& e) {
                    EXPECT_STREQ(e.what(),
                                 KalmanFilterErrors::NonPositiveSemiDefiniteErrorCovariance);
                    throw;
                }
            },
            std::invalid_argument);
    }

    {
        KalmanFilterTestParams<double, 2> params;
        params.errorCovarianceMatrix << 1.0, 5.0, 5.0, 1.0;

        EXPECT_THROW(
            {
                try {
                    KalmanFilterFromParams(params);
                } catch (const std::invalid_argument& e) {
                    EXPECT_STREQ(e.what(),
                                 KalmanFilterErrors::NonPositiveSemiDefiniteErrorCovariance);
                    throw;
                }
            },
            std::invalid_argument);
    }
}

TEST(KalmanFilterInitTests, InitWithNonPositiveSemiDefiniteProcessNoiseMatrixIsError) {
    {
        KalmanFilterTestParams<double, 2> params;
        params.processNoiseMatrix(1, 1) *= -1;

        EXPECT_THROW(
            {
                try {
                    KalmanFilterFromParams(params);
                } catch (const std::invalid_argument& e) {
                    EXPECT_STREQ(e.what(), KalmanFilterErrors::NonPositiveSemiDefiniteProcessNoise);
                    throw;
                }
            },
            std::invalid_argument);
    }

    {
        KalmanFilterTestParams<double, 2> params;
        params.processNoiseMatrix << 1.0, 5.0, 5.0, 1.0;

        EXPECT_THROW(
            {
                try {
                    KalmanFilterFromParams(params);
                } catch (const std::invalid_argument& e) {
                    EXPECT_STREQ(e.what(), KalmanFilterErrors::NonPositiveSemiDefiniteProcessNoise);
                    throw;
                }
            },
            std::invalid_argument);
    }
}
