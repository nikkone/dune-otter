#ifndef FishTagEstimators_MultipleReceiverBase
#define FishTagEstimators_MultipleReceiverBase
#include <Eigen/Core>
#include <OpenFilterPack/UnscentedKalmanFilter.hpp>
#include <OpenFilterPack/AlgebraicSolution.hpp>
#include "Estimator.hpp"
//! Task that runs source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class MultipleReceiverBase : public Estimator<double>
  {
    public:
      typedef enum { 
        param_receiver_depth,
        param_snr_a,
        param_snr_b,
        param_snr_k
      } t_param_base;

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

      virtual void setSNRmodel(double B_fit_inn, double k_fit_inn, double a_fit_inn);

      MultipleReceiverBase() : B_fit(49.807), k_fit(4.9147), a_fit(0.015309) {};
      ~MultipleReceiverBase() {};

      void estimateToStream(std::ostream& os) const;
    protected:
      double receiver_depth;
      //! Model coefficient for SNR = B_fit - k_fit*log(dist) - a_fit*dist
      double B_fit;
      //! Model coefficient for SNR = B_fit - k_fit*log(dist) - a_fit*dist
      double k_fit;
      //! Model coefficient for SNR = B_fit - k_fit*log(dist) - a_fit*dist
      double a_fit;
  };
}
#endif // FishTagEstimators_MultipleReceiverBase