#ifndef FishTagEstimators_SingleReceiverXKF
#define FishTagEstimators_SingleReceiverXKF
#include <Eigen/Core>
#include "SingleReceiverBase.hpp"
#include "XKF/XKF.hpp"

//! Task that runs source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class SingleReceiverXKF : public SingleReceiverBase
  {
    public:
      SingleReceiverXKF() : xkf(false) {};
      ~SingleReceiverXKF() {};

      bool update(TagBuffer *tagBuffer);
      void predict();

      XKF::XKF<double> xkf;
      void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);
      std::tuple<double, double, double> getEstimate();
      void print(std::ostream& os) const;
      bool isActive() const {
        return xkf.stage3.active;
      }

      void setTDOACovariance(double TDOACovariance);
      void setDepthCovariance(double depthCovariance);
      bool checkTime(double transmissionFirstTime,double currentTime) const;
  };
}
#endif // FishTagEstimators_SingleReceiverXKF