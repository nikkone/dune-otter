#ifndef OFP_ExtendedKalmanFilter
#define OFP_ExtendedKalmanFilter
#include <Eigen/Core>
#include "KalmanFilter.hpp"
#include <iostream>
#include <type_traits>
namespace OFP
{
  template <class T, int states, int measurements, int inputs>
  class ExtendedKalmanFilter : public KalmanFilter<T, states, measurements, inputs>
  {
    public:
      //! Holder for Jacobian function
      std::function<Eigen::Matrix<T, measurements, states>(Eigen::Matrix<T, states, 1> x, Eigen::Matrix<T, inputs, 1> u)> calculateJacobian;
      //! Holder for measurment function
      std::function<Eigen::Matrix<T, measurements, 1>(Eigen::Matrix<T, states, 1> x, Eigen::Matrix<T, inputs, 1> u)> h;

      //! Measurment update for the filter. This is dependent on the Jacobian and measurment function being set.
      //! @param[in] yk_in Measurment vector.
      //! @return True if active and end reached, false if filter not active.
/*      
      template <int D = inputs>
      typename std::enable_if<D == 0, Eigen::Matrix<T, measurements, 1>>::type
      calculateInnovation(Eigen::Matrix<T, measurements, 1> yk_in) {
        return yk_in - h(KalmanFilter<T, states, measurements, inputs>::xHat);
      }

      template <int D = inputs>
      typename std::enable_if<D != 0, Eigen::Matrix<T, measurements, 1>>::type
      calculateInnovation(Eigen::Matrix<T, measurements, 1> yk_in, Eigen::Matrix<T, inputs, 1> u) {
        return yk_in - h(KalmanFilter<T, states, measurements, inputs>::xHat, u);
      }
*/

    template <int D = inputs>
    typename std::enable_if<D == 0, bool>::type
    update(Eigen::Matrix<T, measurements, 1> yk_in) {
      if(KalmanFilter<T, states, measurements, inputs>::active) {
        KalmanFilter<T, states, measurements, inputs>::C = calculateJacobian(KalmanFilter<T, states, measurements, inputs>::xHat, Eigen::Matrix<T, inputs, 1>::Zero());
        KalmanFilter<T, states, measurements, inputs>::ykest = h(KalmanFilter<T, states, measurements, inputs>::xHat, Eigen::Matrix<T, inputs, 1>::Zero());
        KalmanFilter<T, states, measurements, inputs>::update(yk_in);
        return true;
      }
      return false;
    }
    
    template <int D = inputs>
    typename std::enable_if<D != 0, bool>::type
    update(Eigen::Matrix<T, measurements, 1> yk_in, Eigen::Matrix<T, inputs, 1> u) {
      if(KalmanFilter<T, states, measurements, inputs>::active) {
        KalmanFilter<T, states, measurements, inputs>::C = calculateJacobian(KalmanFilter<T, states, measurements, inputs>::xHat, u);
        KalmanFilter<T, states, measurements, inputs>::ykest = h(KalmanFilter<T, states, measurements, inputs>::xHat, u);
        KalmanFilter<T, states, measurements, inputs>::update(yk_in, u);
        return true;
      }
      return false;
    }

  };

}
#endif //OFP_ExtendedKalmanFilter