#ifndef FishTagEstimators_SingleReceiverASLV
#define FishTagEstimators_SingleReceiverASLV
//#include <unistd.h>
#include <Eigen/Core>
#include <OpenFilterPack/AlgebraicSolution.hpp>
#include "SingleReceiverBase.hpp"
//! Task that runs source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class SingleReceiverASLV : public SingleReceiverBase
  {
    public:
      bool active;
      bool update(TagBuffer *tagBuffer);
      //void predict();

      OFP::AlgebraicSolver<double, 3, 9, 5> aslv;
      void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);
      std::tuple<double, double, double> getEstimate() const;
      void print(std::ostream& os) const;
      bool isActive() const {
        return active;
      }
      bool activateEstimator() {
        active = true;
        return active;
      };
      /*void setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn);*/
      SingleReceiverASLV() : active(false) {};
      ~SingleReceiverASLV() {};
  };
}
#endif // FishTagEstimators_SingleReceiverASLV