#ifndef OFP_ALGEBRAICSOLVER22_H
#define OFP_ALGEBRAICSOLVER22_H

#include <Eigen/Core>

namespace OFP
{
  template <typename T, int states, int measurements, int neededMeasurements>
  class AlgebraicSolver2 {
    Eigen::Matrix<T, states, 1> refReceiverPos;
    T refReceiverTDOA;
    Eigen::Matrix<T, states, neededMeasurements> posi;
    Eigen::Matrix<T, neededMeasurements, 1> RDoA;
    T depth;
    int receivedMeasurements;
    Eigen::Matrix<T, states, neededMeasurements+1> M;
    Eigen::Matrix<T, neededMeasurements+1,1> D;
  public:
    AlgebraicSolver2();
    Eigen::Matrix<T, states, 1> x; // State Estimation

    bool addMeasurement(Eigen::Matrix<T, measurements, 1> z);
  };
  template class AlgebraicSolver2<double, 3, 8, 2>;
  template class AlgebraicSolver2<double, 3, 8, 3>;
  template class AlgebraicSolver2<double, 3, 8, 4>;
  template class AlgebraicSolver2<double, 3, 8, 5>;
  template class AlgebraicSolver2<double, 3, 8, 6>;
}

#endif //OFP_ALGEBRAICSOLVER22_H