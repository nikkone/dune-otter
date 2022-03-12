#ifndef FishTagEstimators_PeriodEstimator
#define FishTagEstimators_PeriodEstimator
//#include <unistd.h>
#include <Eigen/Core>
#include <OpenFilterPack/UnscentedKalmanFilter.hpp>
#include <OpenFilterPack/AlgebraicSolution.hpp>
#include "Estimator.hpp"
//! Task that runs source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class PeriodEstimator : public Estimator
  {
    public:
      void update(TagBuffer *tagBuffer);
      //void predict();

      void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);
      void print(std::ostream& os) const;
      void parseParameter(unsigned parameterID, double value);

      PeriodEstimator() {};
      ~PeriodEstimator() {};
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
#endif // FishTagEstimators_SingleReceiverEKF