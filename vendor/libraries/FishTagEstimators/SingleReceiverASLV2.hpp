#ifndef FishTagEstimators_SingleReceiverASLV2
#define FishTagEstimators_SingleReceiverASLV2
#include <Eigen/Core>
#include "SingleReceiverBase.hpp"
#include "XKF/LeastSquares.hpp"
//! Task that runs source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class SingleReceiverASLV2 : public SingleReceiverBase
  {
    public:
      bool active;
      bool update(TagBuffer *tagBuffer);
      //void predict();

      XKF::LeastSquares<double> aslv;
      void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);
      std::tuple<double, double, double> getEstimate();
      void print(std::ostream& os) const;
      bool isActive() const {
        return active;
      }
      bool activateEstimator() {
        active = true;
        return active;
      };
      /*void setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn);*/
      SingleReceiverASLV2() : active(false) {};
      ~SingleReceiverASLV2() {};
  };
}
#endif // FishTagEstimators_SingleReceiverASLV2