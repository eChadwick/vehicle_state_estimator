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
inline constexpr const char* NonPositiveSemiDefiniteErrorCovariance{
    "KalmanFilter initialization failed: Non-positve-semi-definite error covariance matrix input"};
inline constexpr const char* NonPositiveSemiDefiniteProcessNoise{
    "KalmanFilter initialization failed: Non-positve-semi-definite process noise matrix input"};
}  // namespace KalmanFilterErrors

template <typename ScalarType, int MatrixDimension>
class KalmanFilter {
  public:
    using Vector = Eigen::Matrix<ScalarType, 1, MatrixDimension>;
    using Matrix = Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>;

    KalmanFilter(const Vector& stateVector, const Matrix& transitionMatrix,
                 const Matrix& errorCovarianceMatrix, const Matrix& processNoiseMatrix) {
        InitStateVector(stateVector);
        InitTransitionMatrix(transitionMatrix);

        InitErrorCovariance(errorCovarianceMatrix);
        InitProcessNoise(processNoiseMatrix);
    }

    const Vector& GetStateVector() const { return mStateVector; }

    const Matrix& GetTransitionMatrix() const { return mTransitionMatrix; }

    const Matrix& GetErrorCovarianceMatrix() const { return mErrorCovarianceMatrix; }

    const Matrix& GetProcessNoiseMatrix() const { return mProcessNoiseMatrix; }

  private:
    void InitStateVector(const Vector& stateVector) {
        if (!stateVector.allFinite()) {
            throw std::invalid_argument(KalmanFilterErrors::NonFiniteStateVector);
        }

        mStateVector = stateVector;
    }

    void InitTransitionMatrix(const Matrix& transitionMatrix) {
        if (!transitionMatrix.allFinite()) {
            throw std::invalid_argument(KalmanFilterErrors::NonFiniteTransitionMatrix);
        }

        mTransitionMatrix = transitionMatrix;
    }

    void InitErrorCovariance(const Matrix& errorCovarianceMatrix) {
        if (!errorCovarianceMatrix.allFinite()) {
            throw std::invalid_argument(KalmanFilterErrors::NonFiniteErrorCovarianceMatrix);
        }

        if (!errorCovarianceMatrix.isApprox(errorCovarianceMatrix.transpose())) {
            throw std::invalid_argument(KalmanFilterErrors::AsymmetricErrorCovariance);
        }

        Eigen::SelfAdjointEigenSolver<Matrix> solver(errorCovarianceMatrix, Eigen::EigenvaluesOnly);
        if (solver.info() != Eigen::Success ||
            solver.eigenvalues().minCoeff() < static_cast<ScalarType>(-1e-6)) {
            throw std::invalid_argument(KalmanFilterErrors::NonPositiveSemiDefiniteErrorCovariance);
        }

        mErrorCovarianceMatrix = errorCovarianceMatrix;
    }

    void InitProcessNoise(const Matrix& processNoiseMatrix) {
        if (!processNoiseMatrix.allFinite()) {
            throw std::invalid_argument(KalmanFilterErrors::NonFiniteProcessNoiseMatrix);
        }

        if (!processNoiseMatrix.isApprox(processNoiseMatrix.transpose())) {
            throw std::invalid_argument(KalmanFilterErrors::AsymmetricProcessNoise);
        }

        Eigen::SelfAdjointEigenSolver<Matrix> solver(processNoiseMatrix, Eigen::EigenvaluesOnly);
        if (solver.info() != Eigen::Success ||
            solver.eigenvalues().minCoeff() < static_cast<ScalarType>(-1e-6)) {
            throw std::invalid_argument(KalmanFilterErrors::NonPositiveSemiDefiniteProcessNoise);
        }

        mProcessNoiseMatrix = processNoiseMatrix;
    }

    Vector mStateVector;
    Matrix mTransitionMatrix;
    Matrix mErrorCovarianceMatrix;
    Matrix mProcessNoiseMatrix;
};