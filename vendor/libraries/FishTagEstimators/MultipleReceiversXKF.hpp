#ifndef FishTagEstimator_MultipleReceiverXKF
#define FishTagEstimator_MultipleReceiverXKF
#include <Eigen/Core>
#include "Estimator.hpp"
#include "XKF/XKF.hpp"

//! Task that runs source position estimation algorithms for TBRFishTag
//! @author Nikolai Lauvås
//! Beregn optimal ref. receiver til stage1 og stage2
//!Fiks kjør update etter tidligst x sec etter første mottatte av sist timestamp slik at den ikke kjører eks. 0-2 fordi 1 ikke har rukket å komme.

namespace FishTagEstimators
{
  template <class T>
  class MultipleReceiverXKF : public Estimator<double>
  {
    public:
      MultipleReceiverXKF() {};
      ~MultipleReceiverXKF() {};

      bool update(TagBuffer *tagBuffer);
      void predict();

      XKF::XKF<T> xkf;
      void initialize(const Eigen::Matrix<T, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<T, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<T, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<T, c_states, 1> &x0_inn);
      std::tuple<T, T, T> getEstimate();
      void print(std::ostream& os) const;
      bool isActive() const {
        return xkf.isInitialized();
      }
  void setTDOACovariance(T TDOACovariance);
  void setDepthCovariance(T depthCovariance);
  bool checkTime(T transmissionFirstTime, T currentTime) const;
    private:
      
  };
    //template class MultipleReceiverXKF<float>;
    template class MultipleReceiverXKF<double>;
    //template class MultipleReceiverXKF<long double>;
}
#endif // FishTagEstimator_MultipleReceiverEKF