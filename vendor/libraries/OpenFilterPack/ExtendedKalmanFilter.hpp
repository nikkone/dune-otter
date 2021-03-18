#ifndef OFP_ExtendedKalmanFilter
#define OFP_ExtendedKalmanFilter
#include <Eigen/Core>
#include "KalmanFilter.hpp"
namespace OFP
{
  class ExtendedKalmanFilter : public KalmanFilter<double, 3, 3>
  {
    public:
      int i;
      void updateCmatrix(Eigen::Matrix<double, 9, 1> zk);
      //Eigen::Matrix<double, 3, 1> getEstimateOfMeasurment();
  };
}
#endif //OFP_ExtendedKalmanFilter