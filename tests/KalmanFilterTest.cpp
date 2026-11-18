#include <Eigen/Dense>
#include <string>

#include "KalmanFilter.hpp"
#include "ZeroAllocationTestFixture.hpp"
#include "gtest/gtest.h"

template <typename ScalarType, int dimension>
using StateVector = Eigen::Matrix<ScalarType, dimension, 1>;

template <typename ScalarType, int dimension>
using Matrix = Eigen::Matrix<ScalarType, dimension, dimension>;

template <typename ScalarType, int StateDimension, int SensorDimension>
struct KalmanFilterTestParams {
    StateVector<ScalarType, StateDimension> stateVector =
        StateVector<ScalarType, StateDimension>::Zero();
    Matrix<ScalarType, StateDimension> transitionMatrix =
        Matrix<ScalarType, StateDimension>::Identity();
    Matrix<ScalarType, StateDimension> errorCovarianceMatrix =
        Matrix<ScalarType, StateDimension>::Identity();
    Matrix<ScalarType, StateDimension> processNoiseMatrix =
        Matrix<ScalarType, StateDimension>::Identity();
    Eigen::Matrix<ScalarType, SensorDimension, StateDimension> measurementMatrix =
        Eigen::Matrix<ScalarType, SensorDimension, StateDimension>::Identity();
};

template <typename ScalarType, int StateDimension, int SensorDimension>
void EXPECT_FILTER_EQ(
    const KalmanFilter<ScalarType, StateDimension, SensorDimension>& filter,
    const KalmanFilterTestParams<ScalarType, StateDimension, SensorDimension>& params) {
    EXPECT_TRUE(params.stateVector.isApprox(filter.GetStateVector()));
    EXPECT_TRUE(params.transitionMatrix.isApprox(filter.GetTransitionMatrix()));
    EXPECT_TRUE(params.errorCovarianceMatrix.isApprox(filter.GetErrorCovarianceMatrix()));
    EXPECT_TRUE(params.processNoiseMatrix.isApprox(filter.GetProcessNoiseMatrix()));
    EXPECT_TRUE(params.measurementMatrix.isApprox(filter.GetMeasurementMatrix()));
}

template <typename ScalarType, int StateDimension, int SensorDimension>
KalmanFilter<ScalarType, StateDimension, SensorDimension> KalmanFilterFromParams(
    KalmanFilterTestParams<ScalarType, StateDimension, SensorDimension> params) {
    return KalmanFilter{params.stateVector, params.transitionMatrix, params.errorCovarianceMatrix,
                        params.processNoiseMatrix, params.measurementMatrix};
};

class KalmanFilterZeroAllocationTestFixture : public ZeroAllocationTestFixture {
    using ZeroAllocationTestFixture::SetUp;
    using ZeroAllocationTestFixture::TearDown;
};

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedScalarTypes) {
    constexpr int StateDimension = 2;
    constexpr int SensorDimension = 2;
    {
        using ScalarType = double;
        KalmanFilterTestParams<ScalarType, StateDimension, SensorDimension> params;
        KalmanFilter<ScalarType, StateDimension, SensorDimension> uUt =
            KalmanFilterFromParams(params);

        EXPECT_FILTER_EQ(uUt, params);
    }

    {
        using ScalarType = float;
        KalmanFilterTestParams<ScalarType, StateDimension, SensorDimension> params;
        KalmanFilter<ScalarType, StateDimension, SensorDimension> uUt =
            KalmanFilterFromParams(params);

        EXPECT_FILTER_EQ(uUt, params);
    }
}

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedStateDimensions) {
    using ScalarType = double;
    constexpr int SensorDimension = 2;

    {
        constexpr int StateDimension = 2;
        KalmanFilterTestParams<ScalarType, StateDimension, SensorDimension> params;
        KalmanFilter<ScalarType, StateDimension, SensorDimension> uUt =
            KalmanFilterFromParams(params);

        EXPECT_FILTER_EQ(uUt, params);
    }

    {
        constexpr int StateDimension = 3;
        KalmanFilterTestParams<ScalarType, StateDimension, SensorDimension> params;
        KalmanFilter<ScalarType, StateDimension, SensorDimension> uUt =
            KalmanFilterFromParams(params);

        EXPECT_FILTER_EQ(uUt, params);
    }
}

TEST_F(KalmanFilterZeroAllocationTestFixture, TemplatizedSensorDimension) {
    using ScalarType = double;
    constexpr int StateDimension = 2;

    {
        constexpr int SensorDimension = 2;
        KalmanFilterTestParams<ScalarType, StateDimension, SensorDimension> params;
        KalmanFilter<ScalarType, StateDimension, SensorDimension> uUt =
            KalmanFilterFromParams(params);

        EXPECT_FILTER_EQ(uUt, params);
    }

    {
        constexpr int SensorDimension = 3;
        KalmanFilterTestParams<ScalarType, StateDimension, SensorDimension> params;
        KalmanFilter<ScalarType, StateDimension, SensorDimension> uUt =
            KalmanFilterFromParams(params);

        EXPECT_FILTER_EQ(uUt, params);
    }
}

TEST_F(KalmanFilterZeroAllocationTestFixture, PredictStep) {
    using ScalarType = double;
    constexpr int StateDimension = 2;
    constexpr int SensorDimension = 2;

    KalmanFilterTestParams<ScalarType, StateDimension, SensorDimension> params;
    params.stateVector << 1, 2;
    // clang-format off
    params.transitionMatrix << 1, 2, 
                               3, 4;
    params.errorCovarianceMatrix << 2, 1,
                                    1, 3;
    params.processNoiseMatrix << 0.5, 0.1,
                                 0.1, 0.25;
    // clang-format on
    KalmanFilter<ScalarType, StateDimension, SensorDimension> uUt = KalmanFilterFromParams(params);

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
    KalmanFilterTestParams<double, 2, 3> params;
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
    KalmanFilterTestParams<double, 2, 3> params;
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
            KalmanFilterTestParams<double, 2, 3> params;
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
            KalmanFilterTestParams<double, 2, 3> params;
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
            KalmanFilterTestParams<double, 2, 3> params;
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
            KalmanFilterTestParams<double, 2, 3> params;
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
        KalmanFilterTestParams<double, 2, 3> params;
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
        KalmanFilterTestParams<double, 2, 3> params;
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
        KalmanFilterTestParams<double, 2, 3> params;
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
        KalmanFilterTestParams<double, 2, 3> params;
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
