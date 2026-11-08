#pragma once

#include <Eigen/Dense>  // IWYU pragma: keep
#include <cassert>

inline constexpr const char* AsymmetricErrorCovarianceError =
    "KalmanFilter initialization failed: Asymmetric error covariance matrix input";

template <typename ScalarType, int MatrixDimension>
class KalmanFilter {
  public:
    KalmanFilter(
        const Eigen::Matrix<ScalarType, 1, MatrixDimension>& stateVector,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& transitionMatrix,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& errorCovarianceMatrix,
        const Eigen::Matrix<ScalarType, MatrixDimension, MatrixDimension>& processNoiseMatrix) {
        if (!(processNoiseMatrix.isApprox(processNoiseMatrix.transpose()) &&
              stateVector.allFinite() && transitionMatrix.allFinite())) {
            throw std::invalid_argument("");
        }

        if (!errorCovarianceMatrix.isApprox(errorCovarianceMatrix.transpose())) {
            throw std::invalid_argument(AsymmetricErrorCovarianceError);
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