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

template <typename ScalarType, int MatrixDimension>
class KalmanFilter {
  public:
    KalmanFilter(
        const Eigen::Matrix<ScalarType, 1, MatrixDimension>& stateVector,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& transitionMatrix,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& errorCovarianceMatrix,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& processNoiseMatrix) {
        if (!stateVector.allFinite()) {
            throw std::invalid_argument(NonFiniteStateVectorError);
        }
        if (!transitionMatrix.allFinite()) {
            throw std::invalid_argument(NonFiniteTransitionMatrixError);
        }
        if (!errorCovarianceMatrix.allFinite()) {
            throw std::invalid_argument(NonFiniteErrorCovarianceMatrixError);
        }
        if (!processNoiseMatrix.allFinite()) {
            throw std::invalid_argument(NonFiniteProcessNoiseMatrixError);
        }

        if (!errorCovarianceMatrix.isApprox(errorCovarianceMatrix.transpose())) {
            throw std::invalid_argument(AsymmetricErrorCovarianceError);
        }
        if (!processNoiseMatrix.isApprox(processNoiseMatrix.transpose())) {
            throw std::invalid_argument(AsymmetricProcessNoiseError);
        }

        mStateVector = stateVector;
        mTransitionMatrix = transitionMatrix;
        mErrorCovarianceMatrix = errorCovarianceMatrix;
        mProcessNoiseMatrix = processNoiseMatrix;
    }

    const Eigen::Matrix<ScalarType, 1, MatrixDimension>& GetStateVector() const {
        return mStateVector;
    }

    const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& GetTransitionMatrix() const {
        return mTransitionMatrix;
    }

    const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& GetErrorCovarianceMatrix()
        const {
        return mErrorCovarianceMatrix;
    }

    const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& GetProcessNoiseMatrix()
        const {
        return mProcessNoiseMatrix;
    }

  private:
    Eigen::Matrix<ScalarType, 1, MatrixDimension> mStateVector;
    Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension> mTransitionMatrix;
    Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension> mErrorCovarianceMatrix;
    Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension> mProcessNoiseMatrix;
};