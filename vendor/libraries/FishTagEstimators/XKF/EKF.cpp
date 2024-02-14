#include "EKF.hpp"
#include <Eigen/Dense>
#include <iostream>
namespace FishTagEstimators
{
  namespace XKF
  {
    /*template <class T>
    EKF<T>::EKF() {

    }*/
template <class T>
bool EKF<T>::constructCandR(TagBuffer *tagBuffer, Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations, Eigen::Matrix<T, 3, 1> xHatInn)
    {
      //std::cout << OFP::KalmanFilterDynamic<T, 3>::C << std::endl;
      
      if(RDOA.rows() < 2) {
        return false;
      }
      
      // Resize matrices to maximum probable
      OFP::KalmanFilterDynamic<T, 3>::C.resize(RDOA.rows()+1,OFP::KalmanFilterDynamic<T, 3>::nx); // Eq (19) Praveen
      OFP::KalmanFilterDynamic<T, 3>::yk.resize(RDOA.rows()+1,1);
      OFP::KalmanFilterDynamic<T, 3>::ykest.resize(RDOA.rows()+1,1); // Eq (18) Praveen
      OFP::KalmanFilterDynamic<T, 3>::innov.resize(RDOA.rows()+1,1);

      uint32_t referenceReceiver = RDOAcombinations.front().second;
      Eigen::Matrix<T, 3, 1> referenceReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[referenceReceiver]->rbegin())->N, (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->E,(tagBuffer->tagBuffer[referenceReceiver]->rbegin())->D);
      T dm = (xHatInn - referenceReceiverNED).norm(); // "Reference"
      
      uint8_t used = 0, combination = 0;
      for(std::vector<std::pair<uint32_t, uint32_t>>::iterator it = RDOAcombinations.begin(); it != RDOAcombinations.end(); it++) {
        if(it->second == referenceReceiver) {

          // Current
          Eigen::Matrix<T, 3, 1> currentReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->first]->rbegin())->N, (tagBuffer->tagBuffer[it->first]->rbegin())->E,(tagBuffer->tagBuffer[it->first]->rbegin())->D);
          T dr = (xHatInn - currentReceiverNED).norm();

          OFP::KalmanFilterDynamic<T, 3>::C.row(used) = 
          (referenceReceiverNED-xHatInn ).transpose()/dm - 
          (currentReceiverNED  -xHatInn ).transpose()/dr;

          OFP::KalmanFilterDynamic<T, 3>::yk.row(used) << RDOA(combination);
          OFP::KalmanFilterDynamic<T, 3>::ykest.row(used) << dr - dm; // Eq. 17 Praveen|| q_t - p_i || - || q_t - p_m ||
          used++;
        }
        combination++;
      }

      // Add depth measurement
      OFP::KalmanFilterDynamic<T, 3>::C.row(used) = OFP::KalmanFilterDynamic<T, 3>::C.row(used).Zero(1,OFP::KalmanFilterDynamic<T, 3>::nx); // May need a +1
      OFP::KalmanFilterDynamic<T, 3>::C.bottomRightCorner(1, 1) << 1.0;
      OFP::KalmanFilterDynamic<T, 3>::ykest.row(used) << xHatInn.row(2);
      OFP::KalmanFilterDynamic<T, 3>::yk.row(used) << (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->trans_data*tagBuffer->getDepthConversionCoefficient(); // Depth measurement
      
      // Resize matrices and vectors according to used
      OFP::KalmanFilterDynamic<T, 3>::R.resize(used+1,used+1);
      OFP::KalmanFilterDynamic<T, 3>::yk.resize(used+1,1);
      OFP::KalmanFilterDynamic<T, 3>::ykest.resize(used+1,1);
      OFP::KalmanFilterDynamic<T, 3>::innov.resize(used+1,1);

      // Compute error from stage 3 and stage 2 estimate
      Eigen::Matrix<T, Eigen::Dynamic,1> error_stage3;
      error_stage3.resize(used+1,1);
      error_stage3 << OFP::KalmanFilterDynamic<T, 3>::C*(OFP::KalmanFilterDynamic<T, 3>::xHat - xHatInn);
      OFP::KalmanFilterDynamic<T, 3>::ykest += error_stage3; // (18) in paper

      // Create R matrix
      OFP::KalmanFilterDynamic<T, 3>::R = OFP::KalmanFilterDynamic<T, 3>::R.Zero(used+1, used+1);
      OFP::KalmanFilterDynamic<T, 3>::R.topLeftCorner(3, 3) << Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Constant(used,used, rr_cov);
      OFP::KalmanFilterDynamic<T, 3>::R.topLeftCorner(used,used) += Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(used,used)*rr_cov;
      OFP::KalmanFilterDynamic<T, 3>::R.bottomRightCorner(1, 1) << rz_cov;

std::cout << "error_stage3" << std::endl << error_stage3 << std::endl;
std::cout << "C" << std::endl << OFP::KalmanFilterDynamic<T, 3>::C << std::endl;
std::cout << "R" << std::endl << OFP::KalmanFilterDynamic<T, 3>::R << std::endl;
std::cout << "yk" << std::endl << OFP::KalmanFilterDynamic<T, 3>::yk << std::endl;
std::cout << "ykest" << std::endl << OFP::KalmanFilterDynamic<T, 3>::ykest << std::endl;

      return true;
    }
  }
}