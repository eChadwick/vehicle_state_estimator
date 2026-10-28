#pragma once

#include <Eigen/Dense>  // IWYU pragma: keep

template <typename ScalarType, int MatrixDimension>
class KalmanFilter {
  public:
    KalmanFilter(
        const Eigen::Matrix<ScalarType, 1, MatrixDimension>& state_vector,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& transitionMatrix,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& errorCovarianceMatrix,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& processNoiseMatrix)
        : mStateVector{state_vector},
          mTransitionMatrix{transitionMatrix},
          mErrorCovarianceMatrix{errorCovarianceMatrix},
          mProcessNoiseMatrix{processNoiseMatrix} {}

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