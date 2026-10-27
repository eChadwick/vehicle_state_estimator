#include "KalmanFilter.hpp"

#include <Eigen/Dense>  // IWYU pragma: keep

KalmanFilter::KalmanFilter(const Eigen::Matrix<double, 1, 2>& stateVector,
                           const Eigen::Matrix<double, 2, 2>& transitionMatrix,
                           const Eigen::Matrix<double, 2, 2>& errorCovarianceMatrix,
                           const Eigen::Matrix<double, 2, 2>& processNoiseMatrix)
    : mStateVector{stateVector},
      mTransitionMatrix{transitionMatrix},
      mErrorCovarianceMatrix{errorCovarianceMatrix},
      mProcessNoiseMatrix{processNoiseMatrix} {}

const Eigen::Matrix<double, 1, 2>& KalmanFilter::GetStateVector() {
    return mStateVector;
}

const Eigen::Matrix<double, 2, 2>& KalmanFilter::GetTransitionMatrix() {
    return mTransitionMatrix;
}

const Eigen::Matrix<double, 2, 2>& KalmanFilter::GetErrorCovarianceMatrix() {
    return mErrorCovarianceMatrix;
}

const Eigen::Matrix<double, 2, 2>& KalmanFilter::GetProcessNoiseMatrix() {
    return mProcessNoiseMatrix;
}