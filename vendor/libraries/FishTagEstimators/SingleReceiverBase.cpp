#include "SingleReceiverBase.hpp"
namespace FishTagEstimators
{
  void SingleReceiverBase::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    name = "SingleReceiverBase";

    registerParameter("receiver", param_receiver);
    registerParameter("receiver_depth", param_receiver_depth);
    registerParameter("tag_period", param_tag_period);
    registerParameter("max_jitter", param_max_jitter);
    registerParameter("max_updates_per_new_measurement", param_max_updates_per_new_measurement);
    registerParameter("max_correction_attempts", param_max_correction_attempts);
    registerParameter("interval_mode", param_interval_mode);
    registerParameter("param_use_aslv", param_use_aslv);

    useAslv = true;

    Estimator::initialize(A_inn, Q_inn, P0_inn, x0_inn);
  }

  void SingleReceiverBase::parseParameter(unsigned parameterID, double value) {
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
      case param_interval_mode:
        interval_mode = value;
        break;
      case param_use_aslv:
        useAslv = (value) ? true : false;
        break;
      default:
        Estimator::parseParameter(parameterID, value);
        break;
    }
  }

  bool SingleReceiverBase::update(TagBuffer *tagBuffer) {
    if(tagBuffer->tagBuffer.find(receiver) !=tagBuffer->tagBuffer.end()) {
      //std::cout << "interval_mode: " << interval_mode << std::endl;
      switch(interval_mode) {
        case interval_mode_fixed_known:
          break;
        case interval_mode_fixed_unknown:
          tag_period = tagBuffer->calculateRegularPeriod(receiver, 5);
          break;
        case interval_mode_irregular_unknown:
          tag_period = tagBuffer->calculateIrregularPeriod(receiver);
          break;
        default:
          std::cout << "Error: Unknown interval mode!" << std::endl;
          return false; // Exit if not in these modes
      }
      //std::cout << "interval_mode: " << interval_mode << std::endl;
      //std::cout << "tag_period: " << tag_period << std::endl;
      return true;
    }
    return false;
  }

  void SingleReceiverBase::estimateToStream(std::ostream& os) const {
    std::tuple<double, double, double>  estimate = getEstimate();
    double result[3] = {std::get<0>(estimate), std::get<1>(estimate), std::get<2>(estimate)};
    os << result[0] << "," << result[1] << "," << result[2];
  }

   std::tuple<double, double, double> SingleReceiverBase::getEstimate() const{
    std::cout << "SingleReceiverBase getEstimate!!!" << std::endl;
    return {0.0,0.0,0.0};
  }
}