#ifndef FishTagEstimators_SingleReceiverBase
#define FishTagEstimators_SingleReceiverBase
#include <Eigen/Core>
#include <OpenFilterPack/UnscentedKalmanFilter.hpp>
#include <OpenFilterPack/AlgebraicSolution.hpp>
#include "Estimator.hpp"
//! Task that runs source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class SingleReceiverBase : public Estimator<double>
  {
    public:
      typedef enum { 
        param_receiver,
        param_receiver_depth,
        param_tag_period,
        param_max_jitter,
        param_max_updates_per_new_measurement,
        param_max_correction_attempts,
        param_interval_mode
      } t_param_base;
      void update(TagBuffer *tagBuffer);

      void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);
      void parseParameter(unsigned parameterID, double value);
      bool isActive() const {
        return false;
      }
      bool activateEstimator() {
        return false;
      };
      SingleReceiverBase() {};
      ~SingleReceiverBase() {};
    protected:
      enum interval_mode_enum
      { 
        interval_mode_fixed_known,
        interval_mode_fixed_unknown,
        interval_mode_irregular_unknown
      };
      unsigned interval_mode;
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
#endif // FishTagEstimators_SingleReceiverBase