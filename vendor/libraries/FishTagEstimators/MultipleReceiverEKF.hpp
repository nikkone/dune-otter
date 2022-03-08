#ifndef FishTag_MultipleReceiverEKF
#define FishTag_MultipleReceiverEKF
#include <DUNE/DUNE.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <unistd.h>
#include <Eigen/Core>
#include <OpenFilterPack/KalmanFilterDynamic.hpp>
#include "Estimator.hpp"
#include <iostream>

//! Task that runst source position estimation algorithms for IMC::TBRFishTag
//! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class MultipleReceiverEKF : public Estimator
  {
    public:
      MultipleReceiverEKF() {};
      ~MultipleReceiverEKF() {};
      void update(const tagBuffer_t &tagBuffer, tagBool_t &unprocessedData);
      void predict();

      OFP::KalmanFilterDynamic<double, c_states> ekf;
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
      
  };
}
#endif // FishTag_MultipleReceiverEKF