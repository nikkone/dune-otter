#include "AlgebraicSolution.hpp"

#include <iostream>
#include <Eigen/Dense>

namespace SourceEstimators
{
  namespace SingleSourceEigen
  {
    template <typename T, int states, int measurements>  AlgebraicSolver<T, states, measurements>::AlgebraicSolver() {
      depth=0.0;
      neededMeasurements = states;
      posi = Eigen::Matrix<T, measurements, states>::Zero();
      ToA = Eigen::Matrix<T, states, 1>::Zero();
      receivedMeasurements = 0;
    }
        template <typename T, int states, int measurements>  
        bool AlgebraicSolver<T, states, measurements>::addMeasurement(Eigen::Matrix<T, measurements, 1> z, Eigen::Matrix<T, 3, 1> position_previous, Eigen::Matrix<T, 3, 1> position_current) {
          depth = z(2);
          T rtoa = z(0);
      if (rtoa == 0) {
          rtoa = 0.00000001;
      }
        ////std::cout << "prePosi = " << std::endl << posi << std::endl;
        ////std::cout << "PreToA = " << std::endl << ToA << std::endl;
      //position_current(2) += ((double) std::rand() / RAND_MAX)/100;
      if (receivedMeasurements == 0) {
          ToA(receivedMeasurements) = 0.0;
          ////std::cout << position_previous << std::endl;
          posi.col(receivedMeasurements) = position_previous;
          ToA(receivedMeasurements) = 0.0;
          receivedMeasurements++;
          posi.col(receivedMeasurements) = position_current;
          ToA(receivedMeasurements) = ToA(receivedMeasurements-1) + rtoa;
          receivedMeasurements++;
      } else {
          ////std::cout << "Measurements" << ((measurements % neededMeasurements) -1) % neededMeasurements << std::endl;
          posi.col(receivedMeasurements % neededMeasurements) = position_current;
          int prevMeasurement = ((receivedMeasurements % neededMeasurements) -1) % neededMeasurements;
          if(prevMeasurement < 0 ) prevMeasurement=neededMeasurements-1;
          ToA(receivedMeasurements % neededMeasurements) = rtoa + ToA(prevMeasurement);
          receivedMeasurements++;
      }
        ////std::cout << "Posi = " << std::endl << posi << std::endl;
        ////std::cout << "ToA = " << std::endl << ToA << std::endl;
      if (receivedMeasurements > 5-1) {
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
    template <typename T, int states, int measurements> 
    bool AlgebraicSolver<T, states, measurements>::solve() {
        //std::cout << " AlgebraicSolver Batch solve time!" << std::endl;
        //std::cout << "Posi = " << std::endl << posi << std::endl;
        //std::cout << "ToA = " << std::endl << ToA << std::endl;
        //double depth = 3.0;
        Eigen::Matrix<T, 3, states+1> M = Eigen::Matrix<T, 3, states+1>::Zero();// arma::zeros<arma::mat>(3,neededMeasurements+1);
        Eigen::Matrix<T, 3, 1> temp={0.0,0.0,1.0};
        Eigen::Matrix<T, states+1,1> D = Eigen::Matrix<T, states+1,1>::Zero();
        M.col(states) = temp;
        D(states) = depth;
        //std::cout << "M = " << std::endl << M << std::endl;
        //std::cout << "D = " << std::endl << D << std::endl;
        //cout << "Starting loop!" << endl;
        for (unsigned int m = 2; m < states; m++) {
        
            T ddm = ToA(m);
            T dd2 = ToA(1);

            M.col(m) = (2*(posi.col(m) - posi.col(0)) / ddm) - (2*(posi.col(1)-posi.col(0)) / dd2);
            //        //std::cout << "part = " << std::endl << (2*(posi.col(m) - posi.col(0)) / ddm) - (2*(posi.col(1)-posi.col(0)) / dd2) << std::endl;

            //M.col(m) = (2*(posi.col(m)));// - posi.col(0)) / ddm) - (2*(posi.col(1)-posi.col(0)) / dd2);
            //std::cout << "M.Col = " << std::endl << M.col(m) << std::endl;
            ////cout << "M.Col" << endl << M.col(m) << endl;
            T off1 = (posi.col(0).cwiseProduct(posi.col(0))).sum();
            T off2 = (posi.col(1).cwiseProduct(posi.col(1))).sum();
            T offm = (posi.col(m).cwiseProduct(posi.col(m))).sum();
        //
            D(m) = ddm - dd2 + (off1-offm)/ddm - (off1-off2)/dd2;
            //std::cout << "Iter " << m << " ok" << std::endl;
        }
        //std::cout << "M = " << std::endl << M << std::endl;
        //std::cout << "D = " << std::endl << D << std::endl;
        //cout << "Extracting from " << 2 << " to " << posi.n_cols-1 << endl;
        //M = M.block(2,3,measurements,3);
        //D = -D.rows(2,measurements);
        //std::cout << "M^T = " << std::endl << M.block(0,2,3,states-1).transpose() << std::endl;
        //std::cout << "-D = " << std::endl << -D.block(2,0,states-1,1) << std::endl;
        //M.colPivHouseholderQr().solve(D);
        x = (M.block(0,2,3,states-1).transpose()).colPivHouseholderQr().solve(-D.block(2,0,states-1,1));// << std::endl; //Eigen::Solve(M.t(),D);
        /*if (x = Eigen::Solve(M.t(),D)) {
            return true;
        }*/
        return false;
    }

  }
}