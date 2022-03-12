#include "PeriodEstimator.hpp"
namespace FishTagEstimators
{
  typedef enum 
  { 
    param_receiver,
    param_receiver_depth,
    param_tag_period,
    param_max_jitter,
    param_max_updates_per_new_measurement,
    param_max_correction_attempts
  } t_param;

  void PeriodEstimator::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    name = "PeriodEstimator";

    registerParameter("receiver");
    registerParameter("receiver_depth");
    registerParameter("tag_period");
    registerParameter("max_jitter");
    registerParameter("max_updates_per_new_measurement");
    registerParameter("max_correction_attempts");

    Estimator::initialize(A_inn, Q_inn, P0_inn, x0_inn);
  }

  void PeriodEstimator::parseParameter(unsigned parameterID, double value) {
    switch(parameterID) {
      case param_receiver:
        receiver = value;
        break;
      case param_receiver_depth:
        receiver_depth = value;
        break;
      case param_tag_period:
        tag_period = value;
        break;
      case param_max_jitter:
        max_jitter = value;
        break;
      case param_max_updates_per_new_measurement:
        max_updates_per_new_measurement = value;
        break;
      case param_max_correction_attempts:
        max_correction_attempts = value;
        break;
      default:
        break;
    }
  }

  void PeriodEstimator::update(TagBuffer *tagBuffer) {
    std::cout << "Estimated period: " << tagBuffer->calculateRegularPeriod(receiver, 5) << std::endl;
  }
  
  //void PeriodEstimator::predict() {
  //  ukf.predict();
  //}
  void PeriodEstimator::print(std::ostream& os) const {
    //os << "A" << std::endl << ukf.A << std::endl;
    //os << "C" << std::endl << ukf.C << std::endl;
    //os << "PHat" << std::endl << ukf.PHat << std::endl;
    //os << "xHat" << std::endl << ukf.xHat << std::endl;
    //os << "R" << std::endl << ukf.R << std::endl;
    //os << "Q" << std::endl << ukf.Q << std::endl;
    //os << "K" << std::endl << ukf.K << std::endl;
    //os << "ykest" << std::endl << ukf.ykest << std::endl;
    //os << "yk" << std::endl << ukf.yk << std::endl;
    //os << "innov" << std::endl << ukf.innov << std::endl;
  }
}