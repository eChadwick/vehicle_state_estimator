#pragma once

#include <Eigen/Dense>  // IWYU pragma: keep
#include <cassert>

namespace KalmanFilterErrors {
inline constexpr const char* AsymmetricErrorCovariance{
    "KalmanFilter initialization failed: Asymmetric error covariance matrix input"};
inline constexpr const char* AsymmetricProcessNoise{
    "KalmanFilter initialization failed: Asymmetric process noise matrix input"};
inline constexpr const char* NonFiniteStateVector{
    "KalmanFilter initialization failed: Non-finite state vector input"};
inline constexpr const char* NonFiniteTransitionMatrix{
    "KalmanFilter initialization failed: Non-finite transition matrix input"};
inline constexpr const char* NonFiniteErrorCovarianceMatrix{
    "KalmanFilter initialization failed: Non-finite transition matrix input"};
inline constexpr const char* NonFiniteProcessNoiseMatrix{
    "KalmanFilter initialization failed: Non-finite process noise matrix input"};
inline constexpr const char* NonFiniteMeasurementMatrix{
    "KalmanFilter initialization failed: Non-finite measurement matrix input"};
inline constexpr const char* NonPositiveSemiDefiniteErrorCovariance{
    "KalmanFilter initialization failed: Non-positve-semi-definite error covariance matrix input"};
inline constexpr const char* NonPositiveSemiDefiniteProcessNoise{
    "KalmanFilter initialization failed: Non-positve-semi-definite process noise matrix input"};
}  // namespace KalmanFilterErrors

template <typename ScalarType, int StateDimension, int SensorDimension>
class KalmanFilter {
  public:
    using StateSpaceVector = Eigen::Matrix<ScalarType, StateDimension, 1>;
    using StateSpaceMatrix = Eigen::Matrix<ScalarType, StateDimension, StateDimension>;

    KalmanFilter(
        const StateSpaceVector& stateVector, const StateSpaceMatrix& transitionMatrix,
        const StateSpaceMatrix& errorCovarianceMatrix, const StateSpaceMatrix& processNoiseMatrix,
        const Eigen::Matrix<ScalarType, SensorDimension, StateDimension>& measurementMatrix,
        const Eigen::Matrix<ScalarType, SensorDimension, SensorDimension>&
            sensorErrorCovarianceMatrix) {
        InitStateVector(stateVector);
        InitTransitionMatrix(transitionMatrix);
        InitErrorCovariance(errorCovarianceMatrix);
        InitProcessNoise(processNoiseMatrix);
        InitMeasurementMatrix(measurementMatrix);
        mSensorErrorCovarianceMatrix = sensorErrorCovarianceMatrix;
    }

    void Predict() {
        mStateVector = mTransitionMatrix * mStateVector;
        mErrorCovarianceMatrix =
            mTransitionMatrix * mErrorCovarianceMatrix * mTransitionMatrix.transpose() +
            mProcessNoiseMatrix;
    }

    const StateSpaceVector& GetStateVector() const { return mStateVector; }

    const StateSpaceMatrix& GetTransitionMatrix() const { return mTransitionMatrix; }

    const StateSpaceMatrix& GetErrorCovarianceMatrix() const { return mErrorCovarianceMatrix; }

    const StateSpaceMatrix& GetProcessNoiseMatrix() const { return mProcessNoiseMatrix; }

    const Eigen::Matrix<ScalarType, SensorDimension, StateDimension>& GetMeasurementMatrix() const {
        return mMeasurementMatrix;
    }

    const Eigen::Matrix<ScalarType, SensorDimension, SensorDimension>&
    GetSensorErrorCovarianceMatrix() const {
        return mSensorErrorCovarianceMatrix;
    }

  private:
    void InitStateVector(const StateSpaceVector& stateVector) {
        if (!stateVector.allFinite()) {
            throw std::invalid_argument(KalmanFilterErrors::NonFiniteStateVector);
        }

        mStateVector = stateVector;
    }

    void InitTransitionMatrix(const StateSpaceMatrix& transitionMatrix) {
        if (!transitionMatrix.allFinite()) {
            throw std::invalid_argument(KalmanFilterErrors::NonFiniteTransitionMatrix);
        }

        mTransitionMatrix = transitionMatrix;
    }

    void InitErrorCovariance(const StateSpaceMatrix& errorCovarianceMatrix) {
        if (!errorCovarianceMatrix.allFinite()) {
            throw std::invalid_argument(KalmanFilterErrors::NonFiniteErrorCovarianceMatrix);
        }

        if (!errorCovarianceMatrix.isApprox(errorCovarianceMatrix.transpose())) {
            throw std::invalid_argument(KalmanFilterErrors::AsymmetricErrorCovariance);
        }

        Eigen::SelfAdjointEigenSolver<StateSpaceMatrix> solver(errorCovarianceMatrix,
                                                               Eigen::EigenvaluesOnly);
        if (solver.info() != Eigen::Success ||
            solver.eigenvalues().minCoeff() < static_cast<ScalarType>(-1e-6)) {
            throw std::invalid_argument(KalmanFilterErrors::NonPositiveSemiDefiniteErrorCovariance);
        }

        mErrorCovarianceMatrix = errorCovarianceMatrix;
    }

    void InitProcessNoise(const StateSpaceMatrix& processNoiseMatrix) {
        if (!processNoiseMatrix.allFinite()) {
            throw std::invalid_argument(KalmanFilterErrors::NonFiniteProcessNoiseMatrix);
        }

        if (!processNoiseMatrix.isApprox(processNoiseMatrix.transpose())) {
            throw std::invalid_argument(KalmanFilterErrors::AsymmetricProcessNoise);
        }

        Eigen::SelfAdjointEigenSolver<StateSpaceMatrix> solver(processNoiseMatrix,
                                                               Eigen::EigenvaluesOnly);
        if (solver.info() != Eigen::Success ||
            solver.eigenvalues().minCoeff() < static_cast<ScalarType>(-1e-6)) {
            throw std::invalid_argument(KalmanFilterErrors::NonPositiveSemiDefiniteProcessNoise);
        }

        mProcessNoiseMatrix = processNoiseMatrix;
    }

    void InitMeasurementMatrix(
        const Eigen::Matrix<ScalarType, SensorDimension, StateDimension>& measurementMatrix) {
        if (!measurementMatrix.allFinite()) {
            throw std::invalid_argument(KalmanFilterErrors::NonFiniteMeasurementMatrix);
        }

        mMeasurementMatrix = measurementMatrix;
    }

    StateSpaceVector mStateVector;
    StateSpaceMatrix mTransitionMatrix;
    StateSpaceMatrix mErrorCovarianceMatrix;
    StateSpaceMatrix mProcessNoiseMatrix;
    Eigen::Matrix<ScalarType, SensorDimension, StateDimension> mMeasurementMatrix;
    Eigen::Matrix<ScalarType, SensorDimension, SensorDimension> mSensorErrorCovarianceMatrix;
};