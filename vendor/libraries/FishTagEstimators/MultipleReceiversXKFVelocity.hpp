#pragma once
#include <Eigen/Core>
#include "Estimator.hpp"
#include "MultipleReceiverBase.hpp"

#include "XKFVelocity/XKF.hpp"

//! Task that runs source position estimation algorithms for TBRFishTag
//! @author Nikolai Lauvås
//! Beregn optimal ref. receiver til stage1 og stage2
//!Fiks kjør update etter tidligst x sec etter første mottatte av sist timestamp slik at den ikke kjører eks. 0-2 fordi 1 ikke har rukket å komme.

namespace FishTagEstimators
{
  template <class T>
  class MultipleReceiverXKFVelocity : public MultipleReceiverBase
  {
    public:
      MultipleReceiverXKFVelocity() {};
      ~MultipleReceiverXKFVelocity() {};

      bool update(TagBuffer *tagBuffer);
      void predict();

      //FishTagEstimators::XKFVelocity::XKF<T> xkf;
      FishTagEstimators::XKFVelocity::XKF<T> xkf;
      void initialize(const Eigen::Matrix<T, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<T, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<T, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<T, c_states, 1> &x0_inn);
      std::tuple<T, T, T> getEstimate() const;
      void print(std::ostream& os) const;

      void estimateToStream(std::ostream& os) const;
      virtual void headerToStream(std::ostream& os) const;

      void setTimestep(T ts) {
        if(ts > 0.0) {
          timestep = ts;
          xkf.setTimestep(ts);
        }
      }


      void setSNRmodel(double B_fit_inn, double k_fit_inn, double a_fit_inn) {
        xkf.stage1.setSNRmodel(B_fit_inn, k_fit_inn, a_fit_inn);
      }

      bool isActive() const {
        return xkf.stage3.active;
      }
  void setTDOACovariance(T TDOACovariance);
  void setDepthCovariance(T depthCovariance);
  bool checkTime(T transmissionFirstTime, T currentTime) const;
    private:
      
  };
    //template class MultipleReceiverXKFVelocity<float>;
    template class MultipleReceiverXKFVelocity<double>;
    //template class MultipleReceiverXKFVelocity<long double>;
}
//#endif // FishTagEstimator_MultipleReceiverXKFVelocity