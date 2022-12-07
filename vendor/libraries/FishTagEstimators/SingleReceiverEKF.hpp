#ifndef FishTagEstimators_SingleReceiverEKF
#define FishTagEstimators_SingleReceiverEKF
//#include <unistd.h>
#include <Eigen/Core>
#include <OpenFilterPack/KalmanFilterDynamic.hpp>
#include <OpenFilterPack/AlgebraicSolution.hpp>
#include "SingleReceiverBase.hpp"
//! Task that runs source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class SingleReceiverEKF : public SingleReceiverBase
  {
    public:
      bool update(TagBuffer *tagBuffer);
      void predict();

      OFP::KalmanFilterDynamic<double, c_states> ekf;
      OFP::AlgebraicSolver<double, 3, 9, 5> aslv;
      void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);
      std::tuple<double, double, double> getEstimate();
      void print(std::ostream& os) const;
      bool isActive() const {
        return ekf.active;
      }
      bool activateEstimator() {
        ekf.active = true;
        return ekf.active;
      };
      void setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn);
      SingleReceiverEKF() {};
      ~SingleReceiverEKF() {};
  };
}
#endif // FishTagEstimators_SingleReceiverEKF