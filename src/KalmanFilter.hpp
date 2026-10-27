#include <Eigen/Dense>  // IWYU pragma: keep

class KalmanFilter {
  public:
    KalmanFilter(const Eigen::Matrix<double, 1, 2>& state_vector,
                 const Eigen::Matrix<double, 2, 2>& transitionMatrix,
                 const Eigen::Matrix<double, 2, 2>& errorCovarianceMatrix,
                 const Eigen::Matrix<double, 2, 2>& processNoiseMatrix);
    const Eigen::Matrix<double, 1, 2>& GetStateVector();
    const Eigen::Matrix<double, 2, 2>& GetTransitionMatrix();
    const Eigen::Matrix<double, 2, 2>& GetErrorCovarianceMatrix();
    const Eigen::Matrix<double, 2, 2>& GetProcessNoiseMatrix();

  private:
    Eigen::Matrix<double, 1, 2> mStateVector;
    Eigen::Matrix<double, 2, 2> mTransitionMatrix;
    Eigen::Matrix<double, 2, 2> mErrorCovarianceMatrix;
    Eigen::Matrix<double, 2, 2> mProcessNoiseMatrix;
};