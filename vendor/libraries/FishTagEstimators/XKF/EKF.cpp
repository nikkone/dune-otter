#include "EKF.hpp"
#include <Eigen/Dense>
#include <iostream>
namespace FishTagEstimators
{
  namespace XKF
  {
    EKF::EKF() {
    }

void EKF::initialize(Eigen::Matrix<double, 3,1> xInit) {
    A << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    Q << qq_cov, 0, 0,
         0, qq_cov, 0,
         0, 0, qq_cov/10;
    xHat << xInit;
    PHat << 100, 0, 0,
            0, 100, 0,
            0, 0, 10;
    inputs = 3;
    D.resize(inputs,inputs);
    D << dt*1, 0, 0,
         0, dt*1, 0,
         0, 0, dt*1;
}

bool EKF::constructCandR(TagBuffer *tagBuffer, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations, Eigen::Matrix<double, 3, 1> xHatInn)
    {
      if(RDOA.rows() < 1) {
        return false;
      }
      
//std::cout << "Entered EKF::constructCandR" <<  std::endl;
      C.resize(RDOA.rows()+1,nx);
      R.resize(RDOA.rows()+1,RDOA.rows()+1);
      yk.resize(RDOA.rows()+1,1);
      ykest.resize(RDOA.rows()+1,1); // From (18) in paper
      innov.resize(RDOA.rows()+1,1);


      yk << RDOA, (tagBuffer->tagBuffer[RDOAcombinations.front().first]->rbegin())->getS256Depth();
      // Create R matrix
      R=R.Constant(RDOA.rows()+1,RDOA.rows()+1,rr_cov);
      R.topLeftCorner(RDOA.rows(),RDOA.rows()) += Eigen::MatrixXd::Identity(RDOA.rows(),RDOA.rows())*rr_cov;
      R.col(RDOA.rows()) << R.col(RDOA.rows()).Zero(RDOA.rows()+1,1); // May need a +1
      R.row(RDOA.rows()) = R.row(RDOA.rows()).Zero(1,RDOA.rows()+1); // May need a +1
      R.bottomRightCorner(1, 1) << rz_cov;

      //Eigen::Matrix<double, Eigen::Dynamic,1> d; // || q_t - p_i ||
      //d.resize(RDOA.rows(),1);
      uint8_t combination = 0;
      for(std::vector<std::pair<uint32_t, uint32_t>>::iterator it = RDOAcombinations.begin(); it != RDOAcombinations.end(); it++) {
        // "Reference"
        double dm = (xHatInn - 
        Eigen::Matrix<double, 3, 1>((tagBuffer->tagBuffer[it->first]->rbegin())->N, (tagBuffer->tagBuffer[it->first]->rbegin())->E,(tagBuffer->tagBuffer[it->first]->rbegin())->D)
        ).norm(); 
        // Current
        double dr = (xHatInn - 
        Eigen::Matrix<double, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D)
        ).norm();

        C.row(combination) = (
        Eigen::Matrix<double, 3, 1>((tagBuffer->tagBuffer[it->first]->rbegin())->N, (tagBuffer->tagBuffer[it->first]->rbegin())->E,(tagBuffer->tagBuffer[it->first]->rbegin())->D)
        -xHatInn ).transpose()/dm - (
        Eigen::Matrix<double, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D)
        -xHatInn ).transpose()/dr;
        ykest.row(combination) << dr - dm; // || q_t - p_i || - || q_t - p_m ||
        combination++;
      }

      



      // Add depth measurement
      C.row(RDOA.rows()) = C.row(RDOA.rows()).Zero(1,nx); // May need a +1
      C.bottomRightCorner(1, 1) << 1.0;
      ykest.row(combination) << xHatInn.row(2);

      // Compute error from stage 3 and stage 2 estimate
      Eigen::Matrix<double, Eigen::Dynamic,1> error_stage3;
      error_stage3.resize(RDOA.rows()+1,1);
      error_stage3 << C*(xHat - xHatInn);


      //ykest.row(RDOA.rows()) << xHatInn(2,0) - error_stage3(RDOA.rows(),0);
      ykest += error_stage3; // (18) in paper

//std::cout << "error_stage3" << std::endl << error_stage3 << std::endl;
//std::cout << "C" << std::endl << C << std::endl;
//std::cout << "R" << std::endl << R << std::endl;
//std::cout << "yk" << std::endl << yk << std::endl;
//std::cout << "ykest" << std::endl << ykest << std::endl;
      return true;
    }
  }
}