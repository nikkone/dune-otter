#ifndef OFP_ExtendedKalmanFilter
#define OFP_ExtendedKalmanFilter
#include <Eigen/Core>
#include "KalmanFilter.hpp"
#include <iostream>
namespace OFP
{
  template <class T, int states, int measurements>
  class ExtendedKalmanFilter : public KalmanFilter<T, states, measurements>
  {
    public:
      std::function<Eigen::Matrix<T, measurements, states>(Eigen::Matrix<T, states, 1> x)> calculateJacobian;
      std::function<Eigen::Matrix<T, measurements, 1>(Eigen::Matrix<T, states, 1> x)> h;
      void update(Eigen::Matrix<T, measurements, 1> z) {
        if(KalmanFilter<T, states, measurements>::active) {
          KalmanFilter<T, states, measurements>::C = calculateJacobian(KalmanFilter<T, states, measurements>::xHat);
          KalmanFilter<T, states, measurements>::ykest = h(KalmanFilter<T, states, measurements>::xHat);
          KalmanFilter<T, states, measurements>::update(z);
        }
      }
  };
}
#endif //OFP_ExtendedKalmanFilter