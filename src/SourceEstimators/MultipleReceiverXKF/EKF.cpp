#include "EKF.hpp"
#include <Eigen/Dense>
#include <iostream>
namespace SourceEstimators
{
  namespace MultipleReceiverXKF
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

bool EKF::constructCandR(Eigen::Matrix<double, 3, Eigen::Dynamic> receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, double tagDepth, Eigen::Matrix<double, 3, 1> xHatInn)
    {
      if(RDOA.rows() < 1) {
        return false;
      }
std::cout << "Entered EKF::constructCandR" <<  std::endl;
      C.resize(RDOA.rows()+1,nx);
      R.resize(RDOA.rows()+1,RDOA.rows()+1);
      yk.resize(receiverPositions.cols(),1);
      ykest.resize(receiverPositions.cols(),1); // From (18) in paper
      innov.resize(receiverPositions.cols(),1);

      yk << RDOA, tagDepth;
      // Create R matrix
      R=R.Constant(RDOA.rows()+1,RDOA.rows()+1,rr_cov);
      R.topLeftCorner(RDOA.rows(),RDOA.rows()) += Eigen::MatrixXd::Identity(RDOA.rows(),RDOA.rows())*rr_cov;
      R.col(RDOA.rows()) << R.col(RDOA.rows()).Zero(RDOA.rows()+1,1); // May need a +1
      R.row(RDOA.rows()) = R.row(RDOA.rows()).Zero(1,RDOA.rows()+1); // May need a +1
      R.bottomRightCorner(1, 1) << rz_cov;

      Eigen::Matrix<double, Eigen::Dynamic,1> d; // || q_t - p_i ||
      d.resize(receiverPositions.cols(),1);
      for(unsigned i = 0;i<receiverPositions.cols();i++) {
        d(i) = (xHatInn - receiverPositions.col(i)).norm();
      }

      // Range measurments
      for(unsigned i = 1;i<receiverPositions.cols();i++) {
        C.row(i-1) = ( receiverPositions.col(0)-xHatInn ).transpose()/d(0) - ( receiverPositions.col(i)-xHatInn ).transpose()/d(i);
        ykest.row(i -1) << d(i) - d(0);
      }


      // Add depth measurement
      C.row(RDOA.rows()) = C.row(RDOA.rows()).Zero(1,nx); // May need a +1
      C.bottomRightCorner(1, 1) << 1.0;
      ykest.row(receiverPositions.cols() -1) << xHatInn.row(2);

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