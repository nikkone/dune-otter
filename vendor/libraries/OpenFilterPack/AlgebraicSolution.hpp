#ifndef OFP_ALGEBRAICSOLVER_H
#define OFP_ALGEBRAICSOLVER_H

#include <Eigen/Core>

namespace OFP
{
  template <typename T, int states, int measurements>
  class AlgebraicSolver {
    Eigen::Matrix<T, 3, states> posi;
    Eigen::Matrix<T, states, 1> ToA;
    T depth;
    int receivedMeasurements;
    int neededMeasurements;
  public:
    AlgebraicSolver();
    Eigen::Matrix<T, states, 1> x; // State Estimation

    bool addMeasurement(Eigen::Matrix<T, measurements, 1> z);
    bool solve();
  };
  template class AlgebraicSolver<double, 5, 9>;
}

#endif //OFP_ALGEBRAICSOLVER_H