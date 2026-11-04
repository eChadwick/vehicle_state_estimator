#pragma once

#include <Eigen/Dense>  // IWYU pragma: keep
#include <cassert>

template <typename ScalarType, int MatrixDimension>
class KalmanFilter {
  public:
    KalmanFilter(
        const Eigen::Matrix<ScalarType, 1, MatrixDimension>& stateVector,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& transitionMatrix,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& errorCovarianceMatrix,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& processNoiseMatrix) {
        assert(stateVector.allFinite());
        assert(transitionMatrix.allFinite());
        assert(errorCovarianceMatrix.isApprox(errorCovarianceMatrix.transpose()));
        assert(processNoiseMatrix.isApprox(processNoiseMatrix.transpose()));

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