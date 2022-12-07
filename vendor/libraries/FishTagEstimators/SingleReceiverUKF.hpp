#ifndef FishTagEstimators_SingleReceiverUKF
#define FishTagEstimators_SingleReceiverUKF
//#include <unistd.h>
#include <Eigen/Core>
#include <OpenFilterPack/UnscentedKalmanFilter.hpp>
#include <OpenFilterPack/AlgebraicSolution.hpp>
#include "SingleReceiverBase.hpp"
//! Task that runs source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class SingleReceiverUKF : public SingleReceiverBase
  {
    public:
      bool update(TagBuffer *tagBuffer);
      void predict();

      OFP::UnscentedKalmanFilter<double,c_states,2> ukf;
      OFP::AlgebraicSolver<double, 3, 9, 5> aslv;
      void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);
      std::tuple<double, double, double> getEstimate();
      void print(std::ostream& os) const;
      bool isActive() const {
        return ukf.active;
      }
      bool activateEstimator() {
        ukf.active = true;
        return ukf.active;
      };
      void setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn);
      SingleReceiverUKF() : ukf(0.001, 2.0, 0.0) {};
      ~SingleReceiverUKF() {};
  };
}
#endif // FishTagEstimators_SingleReceiverEKF