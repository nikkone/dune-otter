#include "AlgebraicSolution2.hpp"

#include <iostream>
#include <Eigen/Dense>

namespace OFP
{
  template <typename T, int states, int measurements, int neededMeasurements>  AlgebraicSolver2<T, states, measurements, neededMeasurements>::AlgebraicSolver2() : 
    refReceiverPos(Eigen::Matrix<T, states, 1>::Zero()),
    refReceiverTDOA(0.0),
    posi(Eigen::Matrix<T, states, neededMeasurements>::Zero()),
    RDoA(Eigen::Matrix<T, neededMeasurements, 1>::Zero()),
    depth(0.0),
    receivedMeasurements(0),
    M(Eigen::Matrix<T, states, neededMeasurements+1>::Zero()),
    D(Eigen::Matrix<T, neededMeasurements+1,1>::Zero()),
    x(Eigen::Matrix<T, states, 1>::Zero())
  {}

    template <typename T, int states, int measurements, int neededMeasurements>  
    bool AlgebraicSolver2<T, states, measurements, neededMeasurements>::addMeasurement(Eigen::Matrix<T, measurements, 1> z) {
        depth =  z(7,0);
        T tdoa = z(6,0);
    if (tdoa == 0) {
        tdoa = 0.00000001;
    }

    

    if (receivedMeasurements == 0) {
        refReceiverPos << z.block(0,0,3,1);
        refReceiverTDOA = tdoa;
        Eigen::Matrix<T, states, 1> temp={0.0,0.0,1.0};
        M.col(neededMeasurements) = temp;
        D(neededMeasurements) = depth;
    }
    M.col(receivedMeasurements) = -2*(z.block(3,0,3,1) - refReceiverPos);
    D(receivedMeasurements) = 1485.0*1485.0*tdoa*tdoa - 1485.0*1485.0*refReceiverTDOA*refReceiverTDOA - z.block(3,0,3,1).squaredNorm() + refReceiverPos.squaredNorm();//;(off1-offm) - (off1-off2);

    receivedMeasurements++;
      ////std::cout << "Posi = " << std::endl << posi << std::endl;
      ////std::cout << "RDoA = " << std::endl << RDoA << std::endl;
    if (receivedMeasurements > neededMeasurements-1) {
      receivedMeasurements = 0;
      std::cout << "M2 = " << std::endl << M << std::endl;
      std::cout << "D2 = " << std::endl << D << std::endl;
        try {
            //solve();
            x = (M.block(0,2,states,neededMeasurements-1).transpose()).colPivHouseholderQr().solve(D.block(2,0,neededMeasurements-1,1));
            return true;
        } catch (...) {
            //cout << "Ah, nope.. exception.." << endl;
            return false;
        }
        
    }
    return false;
      
  }
}