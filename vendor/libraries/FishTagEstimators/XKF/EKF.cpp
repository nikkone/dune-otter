#include "EKF.hpp"
#include <Eigen/Dense>
#include <iostream>
namespace FishTagEstimators
{
  namespace XKF
  {
    template <class T>
    EKF<T>::EKF() {

    }
template <class T>
bool EKF<T>::constructCandR(TagBuffer *tagBuffer, Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations, Eigen::Matrix<T, 3, 1> xHatInn)
    {
      //std::cout << OFP::KalmanFilterDynamic<T, 3>::C << std::endl;
      
      if(RDOA.rows() < 1) {
        return false;
      }
      
//std::cout << "Entered EKF::constructCandR" <<  std::endl;
      OFP::KalmanFilterDynamic<T, 3>::C.resize(RDOA.rows()+1,OFP::KalmanFilterDynamic<T, 3>::nx);
      OFP::KalmanFilterDynamic<T, 3>::R.resize(RDOA.rows()+1,RDOA.rows()+1);
      OFP::KalmanFilterDynamic<T, 3>::yk.resize(RDOA.rows()+1,1);
      OFP::KalmanFilterDynamic<T, 3>::ykest.resize(RDOA.rows()+1,1); // From (18) in paper
      OFP::KalmanFilterDynamic<T, 3>::innov.resize(RDOA.rows()+1,1);


      OFP::KalmanFilterDynamic<T, 3>::yk << RDOA, (tagBuffer->tagBuffer[RDOAcombinations.front().first]->rbegin())->getS256Depth();
      // Create R matrix
      OFP::KalmanFilterDynamic<T, 3>::R=OFP::KalmanFilterDynamic<T, 3>::R.Constant(RDOA.rows()+1,RDOA.rows()+1,rr_cov);
      OFP::KalmanFilterDynamic<T, 3>::R.topLeftCorner(RDOA.rows(),RDOA.rows()) += Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(RDOA.rows(),RDOA.rows())*rr_cov;
      OFP::KalmanFilterDynamic<T, 3>::R.col(RDOA.rows()) << OFP::KalmanFilterDynamic<T, 3>::R.col(RDOA.rows()).Zero(RDOA.rows()+1,1); // May need a +1
      OFP::KalmanFilterDynamic<T, 3>::R.row(RDOA.rows()) = OFP::KalmanFilterDynamic<T, 3>::R.row(RDOA.rows()).Zero(1,RDOA.rows()+1); // May need a +1
      OFP::KalmanFilterDynamic<T, 3>::R.bottomRightCorner(1, 1) << rz_cov;

      //Eigen::Matrix<T, Eigen::Dynamic,1> d; // || q_t - p_i ||
      //d.resize(RDOA.rows(),1);
      uint8_t combination = 0;
      for(std::vector<std::pair<uint32_t, uint32_t>>::iterator it = RDOAcombinations.begin(); it != RDOAcombinations.end(); it++) {
        // "Reference"
        T dm = (xHatInn - 
        Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->first]->rbegin())->N, (tagBuffer->tagBuffer[it->first]->rbegin())->E,(tagBuffer->tagBuffer[it->first]->rbegin())->D)
        ).norm(); 
        // Current
        T dr = (xHatInn - 
        Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D)
        ).norm();

        OFP::KalmanFilterDynamic<T, 3>::C.row(combination) = (
        Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->first]->rbegin())->N, (tagBuffer->tagBuffer[it->first]->rbegin())->E,(tagBuffer->tagBuffer[it->first]->rbegin())->D)
        -xHatInn ).transpose()/dm - (
        Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D)
        -xHatInn ).transpose()/dr;
        OFP::KalmanFilterDynamic<T, 3>::ykest.row(combination) << dr - dm; // || q_t - p_i || - || q_t - p_m ||
        combination++;
      }

      



      // Add depth measurement
      OFP::KalmanFilterDynamic<T, 3>::C.row(RDOA.rows()) = OFP::KalmanFilterDynamic<T, 3>::C.row(RDOA.rows()).Zero(1,OFP::KalmanFilterDynamic<T, 3>::nx); // May need a +1
      OFP::KalmanFilterDynamic<T, 3>::C.bottomRightCorner(1, 1) << 1.0;
      OFP::KalmanFilterDynamic<T, 3>::ykest.row(combination) << xHatInn.row(2);

      // Compute error from stage 3 and stage 2 estimate
      Eigen::Matrix<T, Eigen::Dynamic,1> error_stage3;
      error_stage3.resize(RDOA.rows()+1,1);
      error_stage3 << OFP::KalmanFilterDynamic<T, 3>::C*(OFP::KalmanFilterDynamic<T, 3>::xHat - xHatInn);


      //ykest.row(RDOA.rows()) << xHatInn(2,0) - error_stage3(RDOA.rows(),0);
      OFP::KalmanFilterDynamic<T, 3>::ykest += error_stage3; // (18) in paper

//std::cout << "error_stage3" << std::endl << error_stage3 << std::endl;
//std::cout << "C" << std::endl << C << std::endl;
//std::cout << "R" << std::endl << R << std::endl;
//std::cout << "yk" << std::endl << yk << std::endl;
//std::cout << "ykest" << std::endl << ykest << std::endl;

      return true;
    }
  }
}