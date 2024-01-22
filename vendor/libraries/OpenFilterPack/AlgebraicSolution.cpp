#include "AlgebraicSolution.hpp"

#include <iostream>
#include <Eigen/Dense>

namespace OFP
{
  template <typename T, int states, int measurements, int neededMeasurements>  AlgebraicSolver<T, states, measurements, neededMeasurements>::AlgebraicSolver() : 
    posi(Eigen::Matrix<T, states, neededMeasurements>::Zero()),
    RDoA(Eigen::Matrix<T, neededMeasurements, 1>::Zero()),
    depth(0.0),
    receivedMeasurements(0),
    x(Eigen::Matrix<T, states, 1>::Zero())
  {}

    template <typename T, int states, int measurements, int neededMeasurements>  
    bool AlgebraicSolver<T, states, measurements, neededMeasurements>::addMeasurement(Eigen::Matrix<T, measurements, 1> z) {
        depth =  z(8,0);
        T rdoa = z(6,0);
    if (rdoa == 0) {
        rdoa = 0.00000001;
    }
    if (receivedMeasurements == 0) {
        RDoA(receivedMeasurements) = 0.0;
        ////std::cout << zk.block(0,0,3,1) << std::endl;
        posi.col(receivedMeasurements) = z.block(0,0,3,1);
        RDoA(receivedMeasurements) = 0.0;
        receivedMeasurements++;
        posi.col(receivedMeasurements) = z.block(3,0,3,1);
        RDoA(receivedMeasurements) = RDoA(receivedMeasurements-1) + rdoa;
        receivedMeasurements++;
    } else {
        posi.col(receivedMeasurements % neededMeasurements) = z.block(3,0,3,1);
        int prevMeasurement = ((receivedMeasurements % neededMeasurements) -1) % neededMeasurements;
        if(prevMeasurement < 0 ) prevMeasurement=neededMeasurements-1;
        RDoA(receivedMeasurements % neededMeasurements) = rdoa + RDoA(prevMeasurement);
        receivedMeasurements++;
    }
    if (receivedMeasurements > neededMeasurements-1) {
      receivedMeasurements = 0;
        try {
            solve();
            return true;
        } catch (...) {
            //cout << "Ah, nope.. exception.." << endl;
            return false;
        }
        
    }
    return false;
      
  }
  template <typename T, int states, int measurements, int neededMeasurements>
  bool AlgebraicSolver<T, states, measurements, neededMeasurements>::solve() {

      Eigen::Matrix<T, states, neededMeasurements+1> M = Eigen::Matrix<T, states, neededMeasurements+1>::Zero();// arma::zeros<arma::mat>(3,neededMeasurements+1);
      Eigen::Matrix<T, states, 1> temp={0.0,0.0,1.0};
      Eigen::Matrix<T, neededMeasurements+1,1> D = Eigen::Matrix<T, neededMeasurements+1,1>::Zero();
      M.col(neededMeasurements) = temp;
      D(neededMeasurements) = -depth;

      for (unsigned int m = 2; m < neededMeasurements; m++) {
        
          T ddm = RDoA(m);
          T dd2 = RDoA(1);

          M.col(m) = (2*(posi.col(m) - posi.col(0)) / ddm) - (2*(posi.col(1)-posi.col(0)) / dd2);

          T off1 = (posi.col(0).cwiseProduct(posi.col(0))).sum();
          T off2 = (posi.col(1).cwiseProduct(posi.col(1))).sum();
          T offm = (posi.col(m).cwiseProduct(posi.col(m))).sum();

          D(m) = ddm - dd2 + (off1-offm)/ddm - (off1-off2)/dd2;
      }
      std::cout << "M = " << std::endl << M << std::endl;
      std::cout << "D = " << std::endl << D << std::endl;
      x = (M.block(0,2,states,neededMeasurements-1).transpose()).colPivHouseholderQr().solve(-D.block(2,0,neededMeasurements-1,1));
      return false;
  }

}