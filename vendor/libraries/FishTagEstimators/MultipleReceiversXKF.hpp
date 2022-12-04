#ifndef FishTagEstimator_MultipleReceiverXKF
#define FishTagEstimator_MultipleReceiverXKF
#include <Eigen/Core>
#include <OpenFilterPack/KalmanFilterDynamic.hpp>
#include "Estimator.hpp"
#include "XKF/XKF.hpp"

//! Task that runs source position estimation algorithms for TBRFishTag
//! @author Nikolai Lauvås
//! Beregn optimal ref. receiver til stage1 og stage2
//!Fiks kjør update etter tidligst x sec etter første mottatte av sist timestamp slik at den ikke kjører eks. 0-2 fordi 1 ikke har rukket å komme.

namespace FishTagEstimators
{
  class MultipleReceiverXKF : public Estimator
  {
    public:
      MultipleReceiverXKF() {};
      ~MultipleReceiverXKF() {};
      void update(TagBuffer *tagBuffer);

      void predict();

      XKF::XKF xkf;
      void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);
      std::tuple<double, double, double> getEstimate();
      void print(std::ostream& os) const;
      bool isActive() const {
        return xkf.isInitialized();
        return true;
      }
  void setTDOACovariance(double TDOACovariance);
  void setDepthCovariance(double depthCovariance);
    private:
      
  };
}
#endif // FishTagEstimator_MultipleReceiverEKF