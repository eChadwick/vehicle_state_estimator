#include <Eigen/Dense>  // IWYU pragma: keep

template <typename ScalarType>
class KalmanFilter {
  public:
    KalmanFilter(const Eigen::Matrix<ScalarType, 1, 2>& state_vector,
                 const Eigen::Matrix<ScalarType, 2, 2>& transitionMatrix,
                 const Eigen::Matrix<ScalarType, 2, 2>& errorCovarianceMatrix,
                 const Eigen::Matrix<ScalarType, 2, 2>& processNoiseMatrix);
    const Eigen::Matrix<ScalarType, 1, 2>& GetStateVector();
    const Eigen::Matrix<ScalarType, 2, 2>& GetTransitionMatrix();
    const Eigen::Matrix<ScalarType, 2, 2>& GetErrorCovarianceMatrix();
    const Eigen::Matrix<ScalarType, 2, 2>& GetProcessNoiseMatrix();

  private:
    Eigen::Matrix<ScalarType, 1, 2> mStateVector;
    Eigen::Matrix<ScalarType, 2, 2> mTransitionMatrix;
    Eigen::Matrix<ScalarType, 2, 2> mErrorCovarianceMatrix;
    Eigen::Matrix<ScalarType, 2, 2> mProcessNoiseMatrix;
};