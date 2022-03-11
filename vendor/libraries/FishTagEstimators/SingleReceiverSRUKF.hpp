#ifndef FishTagEstimators_SingleReceiverSRUKF
#define FishTagEstimators_SingleReceiverSRUKF
//#include <unistd.h>
#include <Eigen/Core>
#include <OpenFilterPack/SquareRootUnscentedKalmanFilter.hpp>
#include <OpenFilterPack/AlgebraicSolution.hpp>
#include "Estimator.hpp"
//! Task that runs source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class SingleReceiverSRUKF : public Estimator
  {
    public:
      void update(TagBuffer *tagBuffer);
      void predict();

      OFP::SquareRootUnscentedKalmanFilter<double,c_states,2> srukf;
      OFP::AlgebraicSolver<double, 3, 9, 5> aslv;
      void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);
      std::tuple<double, double, double> getEstimate();
      void print(std::ostream& os) const;
      void parseParameter(unsigned parameterID, double value);
      bool isActive() const {
        return srukf.active;
      }
      bool activateEstimator() {
        srukf.active = true;
        return srukf.active;
      };
      void setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn);
      SingleReceiverSRUKF() : srukf(0.001, 2.0, 0.0) {};
      ~SingleReceiverSRUKF() {};
    private:
      uint32_t receiver;
      double receiver_depth;
      float tag_period;
      float max_jitter;
      Eigen::Matrix<double, 3, 1> pos_current;
      Eigen::Matrix<double, 3, 1> pos_previous;
      unsigned int max_updates_per_new_measurement;
      unsigned int max_correction_attempts;
  };
}
#endif // FishTagEstimators_SingleReceiverSRUKF
