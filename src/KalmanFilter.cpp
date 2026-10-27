#include "KalmanFilter.hpp"

#include <Eigen/Dense>  // IWYU pragma: keep

template <typename ScalarType>
KalmanFilter<ScalarType>::KalmanFilter(const Eigen::Matrix<ScalarType, 1, 2>& stateVector,
                                       const Eigen::Matrix<ScalarType, 2, 2>& transitionMatrix,
                                       const Eigen::Matrix<ScalarType, 2, 2>& errorCovarianceMatrix,
                                       const Eigen::Matrix<ScalarType, 2, 2>& processNoiseMatrix)
    : mStateVector{stateVector},
      mTransitionMatrix{transitionMatrix},
      mErrorCovarianceMatrix{errorCovarianceMatrix},
      mProcessNoiseMatrix{processNoiseMatrix} {}

template <typename ScalarType>
const Eigen::Matrix<ScalarType, 1, 2>& KalmanFilter<ScalarType>::GetStateVector() {
    return mStateVector;
}

template <typename ScalarType>
const Eigen::Matrix<ScalarType, 2, 2>& KalmanFilter<ScalarType>::GetTransitionMatrix() {
    return mTransitionMatrix;
}

template <typename ScalarType>
const Eigen::Matrix<ScalarType, 2, 2>& KalmanFilter<ScalarType>::GetErrorCovarianceMatrix() {
    return mErrorCovarianceMatrix;
}

template <typename ScalarType>
const Eigen::Matrix<ScalarType, 2, 2>& KalmanFilter<ScalarType>::GetProcessNoiseMatrix() {
    return mProcessNoiseMatrix;
}

template class KalmanFilter<float>;
template class KalmanFilter<double>;