#include "LTVKF.hpp"
#include <iostream>
namespace SourceEstimators
{
  namespace MultipleReceiverXKF
  {
    LTVKF::LTVKF() {
    }

void LTVKF::initialize(Eigen::Matrix<double, 3,1> xInit) {
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


bool LTVKF::constructCandR(Eigen::Matrix<double, 3,Eigen::Dynamic> receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, double tagDepth, double m_dr)
    {
//std::cout << "RDOA.rows()" << std::endl << RDOA.rows() << std::endl;
      if(RDOA.rows() < 2) {
        return false;
      }
      //std::cout << "Enter LTVKF::constructCandR" << std::endl;
      C.resize(RDOA.rows()+1,nx);
      R.resize(RDOA.rows()+1,RDOA.rows()+1);
      yk.resize(RDOA.rows()+1,1);
      ykest.resize(RDOA.rows()+1,1);
      innov.resize(RDOA.rows()+1,1);

      // Compute C and R matrices
      //for(unsigned i = 0;i<receiverPositions.rows()-1;i++) {
      //  C.row(i) = ( receiverPositions.row(receiverPositions.rows()-1)-xHatInn.transpose() ).transpose()/d(receiverPositions.rows()-1) - ( receiverPositions.row(i)-xHatInn.transpose() ).transpose()/d(i);
      //}
      //C.row(RDOA.rows()) = C.row(RDOA.rows()).Zero(1,3); // May need a +1
      //C.bottomRightCorner(1, 1) << 0.5;
        C << -(receiverPositions.col(1) - receiverPositions.col(0)).transpose(),
             -(receiverPositions.col(2) - receiverPositions.col(0)).transpose(),
                0,                0, 0.5;

        R = R.Zero(RDOA.rows()+1,RDOA.rows()+1);

        
        Eigen::Matrix<double, 3,1> d; // nja in paper
        d << RDOA(0), RDOA(1), 0;
        //double d1 = -RDOA(2,0); // rim/nja
        //double d2 = RDOA(1, 0); // rjm/nja
        //double z1 = 0.5*(d(1)*d(1) - receiverPositions(0,0)*receiverPositions(0,0) - receiverPositions(1,0)*receiverPositions(1,0) + (receiverPositions(0,2))*(receiverPositions(0,2)) + (receiverPositions(1,2))*(receiverPositions(1,2)));
        //double z2 = 0.5*(d(2)*d(2) - receiverPositions(0,1)*receiverPositions(0,1) - receiverPositions(1,1)*receiverPositions(1,1) + (receiverPositions(0,2))*(receiverPositions(0,2)) + (receiverPositions(1,2))*(receiverPositions(1,2)));

        Eigen::Matrix<double, 3,1> z;

        z << (RDOA(0)*RDOA(0) - receiverPositions.col(1).squaredNorm() + receiverPositions.col(0).squaredNorm()),
             (RDOA(1)*RDOA(1) - receiverPositions.col(2).squaredNorm() + receiverPositions.col(0).squaredNorm()),
             tagDepth;
        
        R(0, 0) = (1/2)*pow(2*rr_cov,2) + (d(0) + m_dr)*(d(0) + m_dr)*2*rr_cov;
        R(1, 1) = (1/2)*pow(2*rr_cov,2) + (d(0) + m_dr)*(d(1) + m_dr)*2*rr_cov;
        R(0,1) = 0.5*pow(rr_cov,2) + (d(0)*d(1) + m_dr*(d(0)+ d(1)) + m_dr*m_dr)*rr_cov;
        R(1,0) = 0.5*pow(rr_cov,2) + (d(0)*d(1) + m_dr*(d(0)+ d(1)) + m_dr*m_dr)*rr_cov;
        R(2,2) = rz_cov;
        // yk = zk from paper, see "new measurement vector" under (15)
        //yk(0, 0) = z.row(0)(0) + m_dr*d(0);
        //yk(1, 0) = z.row(1)(0) + m_dr*d(1);
        //yk(2, 0) = tagDepth/2; // Why divided by two?
        yk = 0.5*z+m_dr*d;
        //Eigen::Matrix<double, 3,1> Yest =  C*xHat;
        //ykest(0, 0) = Yest(0, 0);
        //ykest(1, 0) = Yest(1, 0);
        //ykest(2, 0) = Yest(2, 0);
        ykest = C*xHat; // (15) from paper

//std::cout << "C" << std::endl << C << std::endl;
//std::cout << "R" << std::endl << R << std::endl;
//std::cout << "yk" << std::endl << yk << std::endl;
//std::cout << "ykest" << std::endl << ykest << std::endl;
        return true;
    }
  }
}