#include "MultipleReceiverBase.hpp"
namespace FishTagEstimators
{
  void MultipleReceiverBase::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
   {
    name = "MultipleReceiverBase";

    //registerParameter("receiver", param_receiver);
    registerParameter("receiver_depth", param_receiver_depth);
    registerParameter("snr_a", param_snr_a);
    registerParameter("snr_b", param_snr_b);
    registerParameter("snr_k", param_snr_k);


    Estimator::initialize(A_inn, Q_inn, P0_inn, x0_inn);
  }

  void MultipleReceiverBase::parseParameter(unsigned parameterID, double value) {
    switch(parameterID) {
      case param_receiver_depth:
        receiver_depth = value;
        break;

      case param_snr_a:
        a_fit = value;
        setSNRmodel(B_fit, k_fit, a_fit);
        break;

      case param_snr_b:
        B_fit = value;
        setSNRmodel(B_fit, k_fit, a_fit);
        break;

      case param_snr_k:
        k_fit = value;
        setSNRmodel(B_fit, k_fit, a_fit);
        break;

      default:
        Estimator::parseParameter(parameterID, value);
        break;
    }
  }

  void MultipleReceiverBase::estimateToStream(std::ostream& os) const {
    std::tuple<double, double, double>  estimate = getEstimate();
    double result[3] = {std::get<0>(estimate), std::get<1>(estimate), std::get<2>(estimate)};
    os << result[0] << "," << result[1] << "," << result[2];
  }

  void MultipleReceiverBase::setSNRmodel(double B_fit_inn, double k_fit_inn, double a_fit_inn) {
    std::cout << "SNR update in multisim base" << std::endl;
  }
}