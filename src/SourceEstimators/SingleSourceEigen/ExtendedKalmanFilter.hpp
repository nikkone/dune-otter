#ifndef DUNE_SOURCEESTIMATORS_EIGEN_EKF
#define DUNE_SOURCEESTIMATORS_EIGEN_EKF
#include <Eigen/Core>
#include "KalmanFilter.hpp"
  namespace SourceEstimators
  {
      namespace SingleSourceEigen
      {
        class ExtendedKalmanFilter : public KalmanFilter<double, 3, 3>
        {
          public:
          int i;
          void updateCmatrix(Eigen::Matrix<double, 3, 1> zk, Eigen::Matrix<double, 3, 1> position_previous, Eigen::Matrix<double, 3, 1> position_current);
          Eigen::Matrix<double, 3, 1> getEstimateOfMeasurment();
        };
      }
  }
#endif //DUNE_SOURCEESTIMATORS_EIGEN_EKF