#ifndef FishTag_SingleReceiverEKF
#define FishTag_SingleReceiverEKF
#include <DUNE/DUNE.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <unistd.h>
#include <Eigen/Core>
#include <OpenFilterPack/KalmanFilterDynamic.hpp>
#include <OpenFilterPack/AlgebraicSolution.hpp>
#include "Estimator.hpp"
#include <iostream>
namespace SourceEstimators
{
  //! Task that runs source position estimation algorithms for IMC::TBRFishTag
  //! @author Nikolai Lauvås
  namespace FishTag
  {
    class SingleReceiverEKF : public Estimator
    {
      public:
        SingleReceiverEKF() {};
        ~SingleReceiverEKF() {};
        void update(const tagBuffer_t &tagBuffer, tagBool_t &unprocessedData);
        void predict();

        OFP::KalmanFilterDynamic<double, c_states> ekf;
        OFP::AlgebraicSolver<double, 3, 9, 5> aslv;
        void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                         const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                         const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                         const Eigen::Matrix<double, c_states, 1> &x0_inn);
        std::tuple<double, double, double> getEstimate();
        void print(std::ostream& os) const;
        bool isActive() const {
          return ekf.active;
        }
      private:
        uint32_t receiver;
        double receiver_depth;
        float tag_period;
        float max_jitter;
        unsigned int max_updates_per_new_measurement;
        unsigned int max_correction_attempts;

    };
  }
}
#endif // FishTag_SingleReceiverEKF