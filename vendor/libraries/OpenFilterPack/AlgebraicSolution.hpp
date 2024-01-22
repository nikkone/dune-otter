#ifndef OFP_ALGEBRAICSOLVER_H
#define OFP_ALGEBRAICSOLVER_H

#include <Eigen/Core>

namespace OFP
{
  template <typename T, int states, int measurements, int neededMeasurements>
  class AlgebraicSolver {
    Eigen::Matrix<T, states, neededMeasurements> posi;
    Eigen::Matrix<T, neededMeasurements, 1> RDoA;
    T depth;
    int receivedMeasurements;
  public:
    AlgebraicSolver();
    Eigen::Matrix<T, states, 1> x; // State Estimation

    bool addMeasurement(Eigen::Matrix<T, measurements, 1> z);
    bool solve();
  };
  template class AlgebraicSolver<double, 3, 9, 5>;
}

#endif //OFP_ALGEBRAICSOLVER_H