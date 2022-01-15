#ifndef OFP_ExtendedKalmanFilterDynamic
#define OFP_ExtendedKalmanFilterDynamic
#include <Eigen/Core>
#include "KalmanFilterDynamic.hpp"
#include <iostream>
#include <type_traits>
namespace OFP
{
  template <class T, int states>
  class ExtendedKalmanFilterDynamic : public KalmanFilterDynamic<T, states>
  {
    public:
      //! Holder for Jacobian function
      std::function<Eigen::Matrix<T, Eigen::Dynamic,states>(Eigen::Matrix<T, states, 1> x, Eigen::Matrix<T, Eigen::Dynamic, 1> u)> calculateJacobian;
      //! Holder for measurment function
      std::function<Eigen::Matrix<T, Eigen::Dynamic, 1>(Eigen::Matrix<T, states, 1> x, Eigen::Matrix<T, Eigen::Dynamic, 1> u)> h;

      //! Measurment update for the filter. This is dependent on the Jacobian and measurment function being set.
      //! @param[in] yk_in Measurment vector.
      //! @return True if active and end reached, false if filter not active.
    

    bool update(Eigen::Matrix<T, Eigen::Dynamic, 1> yk_in, Eigen::Matrix<T, Eigen::Dynamic, 1> u) {
        std::cout << "Yk" << std::endl << yk_in << std::endl;
        
        std::cout << "u" << std::endl << u << std::endl;
      if(KalmanFilterDynamic<T, states>::active) {
        //KalmanFilterDynamic<T, states>::C.resize(u.rows(), states);
        KalmanFilterDynamic<T, states>::C = calculateJacobian(KalmanFilterDynamic<T, states>::xHat, u);
        //KalmanFilterDynamic<T, states>::ykest = h(KalmanFilterDynamic<T, states>::xHat, u);
        //KalmanFilterDynamic<T, states>::update(yk_in, u);
        std::cout << "C" << std::endl << KalmanFilterDynamic<T, states>::C << std::endl;
        return true;
      }
      return false;
    }

  };

}
#endif //OFP_ExtendedKalmanFilterDynamic