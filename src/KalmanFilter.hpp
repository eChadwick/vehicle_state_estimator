#pragma once

#include <Eigen/Dense>  // IWYU pragma: keep
#include <cassert>

inline constexpr const char* AsymmetricErrorCovarianceError{
    "KalmanFilter initialization failed: Asymmetric error covariance matrix input"};
inline constexpr const char* AsymmetricProcessNoiseError{
    "KalmanFilter initialization failed: Asymmetric process noise matrix input"};
inline constexpr const char* NonFiniteStateVectorError{
    "KalmanFilter initialization failed: Non-finite state vector input"};
inline constexpr const char* NonFiniteTransitionMatrixError{
    "KalmanFilter initialization failed: Non-finite transition matrix input"};
inline constexpr const char* NonFiniteErrorCovarianceMatrixError{
    "KalmanFilter initialization failed: Non-finite transition matrix input"};
inline constexpr const char* NonFiniteProcessNoiseMatrixError{
    "KalmanFilter initialization failed: Non-finite process noise matrix input"};
inline constexpr const char* NonPositiveSemiDefiniteErrorCovarianceError{
    "KalmanFilter initialization failed: Non-positve-semi-definite error covariance matrix input"};

template <typename ScalarType, int MatrixDimension>
class KalmanFilter {
  public:
    using Vector = Eigen::Matrix<ScalarType, 1, MatrixDimension>;
    using Matrix = Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>;

    KalmanFilter(const Vector& stateVector, const Matrix& transitionMatrix,
                 const Matrix& errorCovarianceMatrix, const Matrix& processNoiseMatrix) {
        if (!stateVector.allFinite()) {
            throw std::invalid_argument(NonFiniteStateVectorError);
        }
        if (!transitionMatrix.allFinite()) {
            throw std::invalid_argument(NonFiniteTransitionMatrixError);
        }

        InitErrorCovariance(errorCovarianceMatrix);
        InitProcessNoise(processNoiseMatrix);
        mStateVector = stateVector;
        mTransitionMatrix = transitionMatrix;
    }

    const Vector& GetStateVector() const { return mStateVector; }

    const Matrix& GetTransitionMatrix() const { return mTransitionMatrix; }

    const Matrix& GetErrorCovarianceMatrix() const { return mErrorCovarianceMatrix; }

    const Matrix& GetProcessNoiseMatrix() const { return mProcessNoiseMatrix; }

  private:
    void InitErrorCovariance(const Matrix& errorCovarianceMatrix) {
        if (!errorCovarianceMatrix.allFinite()) {
            throw std::invalid_argument(NonFiniteErrorCovarianceMatrixError);
        }

        if (!errorCovarianceMatrix.isApprox(errorCovarianceMatrix.transpose())) {
            throw std::invalid_argument(AsymmetricErrorCovarianceError);
        }

        Eigen::SelfAdjointEigenSolver<Matrix> solver(errorCovarianceMatrix, Eigen::EigenvaluesOnly);
        if (solver.info() != Eigen::Success ||
            solver.eigenvalues().minCoeff() < static_cast<ScalarType>(-1e-6)) {
            throw std::invalid_argument(NonPositiveSemiDefiniteErrorCovarianceError);
        }

        mErrorCovarianceMatrix = errorCovarianceMatrix;
    }

    void InitProcessNoise(const Matrix& processNoiseMatrix) {
        if (!processNoiseMatrix.allFinite()) {
            throw std::invalid_argument(NonFiniteProcessNoiseMatrixError);
        }

        if (!processNoiseMatrix.isApprox(processNoiseMatrix.transpose())) {
            throw std::invalid_argument(AsymmetricProcessNoiseError);
        }

        mProcessNoiseMatrix = processNoiseMatrix;
    }

    Vector mStateVector;
    Matrix mTransitionMatrix;
    Matrix mErrorCovarianceMatrix;
    Matrix mProcessNoiseMatrix;
};