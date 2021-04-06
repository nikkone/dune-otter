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
      //! Holder for Jacobian function
      std::function<Eigen::Matrix<T, measurements, states>(Eigen::Matrix<T, states, 1> x)> calculateJacobian;
      //! Holder for measurment function
      std::function<Eigen::Matrix<T, measurements, 1>(Eigen::Matrix<T, states, 1> x)> h;

      //! Measurment update for the filter. This is dependent on the Jacobian and measurment function being set.
      //! @param[in] yk_in Measurment vector.
      //! @return True if active and end reached, false if filter not active.
      bool update(Eigen::Matrix<T, measurements, 1> yk_in) {
        if(KalmanFilter<T, states, measurements>::active) {
          KalmanFilter<T, states, measurements>::C = calculateJacobian(KalmanFilter<T, states, measurements>::xHat);
          KalmanFilter<T, states, measurements>::ykest = h(KalmanFilter<T, states, measurements>::xHat);
          KalmanFilter<T, states, measurements>::update(yk_in);
          return true;
        }
        return false;
      }
  };
}
#endif //OFP_ExtendedKalmanFilter