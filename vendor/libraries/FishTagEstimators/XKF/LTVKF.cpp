#include "LTVKF.hpp"
#include <iostream>
namespace FishTagEstimators
{
  namespace XKF
  {
    template <class T>
    LTVKF<T>::LTVKF() {
      OFP::KalmanFilterDynamic<T, 3>();
    }
template <class T>
bool LTVKF<T>::constructCandR(const TagBuffer *tagBuffer, const Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, const std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations, T dm)
    {
      if(RDOA.rows() < 2) {
        return false;
      }

      // Resize matrices to maximum probable
      OFP::KalmanFilterDynamic<T, 3>::C.resize(RDOA.rows()+1,OFP::KalmanFilterDynamic<T, 3>::nx);
      Eigen::Matrix<T, Eigen::Dynamic,1> d; // nja in paper
      d.resize(RDOA.rows()+1,1);

      Eigen::Matrix<T, Eigen::Dynamic,1> z; // y in paper
      z.resize(RDOA.rows()+1,1); // Y from paper ( applied Eq (8) in paper)

      // Set reference receiver
      uint32_t referenceReceiver = RDOAcombinations.front().second;
      Eigen::Matrix<T, 3, 1> referenceReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer.at(referenceReceiver)->rbegin())->N, (tagBuffer->tagBuffer.at(referenceReceiver)->rbegin())->E,(tagBuffer->tagBuffer.at(referenceReceiver)->rbegin())->D);

      uint8_t used = 0, combination = 0;
      for(auto it : RDOAcombinations) {
        if(it.second == referenceReceiver) {
          // Current
          Eigen::Matrix<T, 3, 1> currentReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer.at(it.first)->rbegin())->N, (tagBuffer->tagBuffer.at(it.first)->rbegin())->E,(tagBuffer->tagBuffer.at(it.first)->rbegin())->D);
          OFP::KalmanFilterDynamic<T, 3>::C.row(used) << -(
          currentReceiverNED -
          referenceReceiverNED).transpose();
          // Note: SquaredNorm(x)=||x||^2
          z.row(used) << RDOA(combination)*RDOA(combination) -  
          currentReceiverNED.squaredNorm() + 
          referenceReceiverNED.squaredNorm();

          d.row(used) << RDOA(combination);
          used++;
        }
        if(used == 2) {
            break;
        }
        combination++;
      }
      if(used < 2) {
        return false;
      }
      // Add depth measurement
      OFP::KalmanFilterDynamic<T, 3>::C.row(used) = OFP::KalmanFilterDynamic<T, 3>::C.row(used).Zero(1,OFP::KalmanFilterDynamic<T, 3>::nx);
      OFP::KalmanFilterDynamic<T, 3>::C.bottomRightCorner(1, 1) << 0.5;
      z.row(used) << (tagBuffer->tagBuffer.at(referenceReceiver)->rbegin())->trans_data*tagBuffer->getDepthConversionCoefficient(); // Depth measurement
      d.row(used) << 0;

      // Resize matrices and vectors according to used
      OFP::KalmanFilterDynamic<T, 3>::C.conservativeResize(used+1,OFP::KalmanFilterDynamic<T, 3>::nx);
      OFP::KalmanFilterDynamic<T, 3>::R.conservativeResize(used+1,used+1);
      OFP::KalmanFilterDynamic<T, 3>::yk.conservativeResize(used+1,1);
      OFP::KalmanFilterDynamic<T, 3>::ykest.conservativeResize(used+1,1);
      OFP::KalmanFilterDynamic<T, 3>::innov.conservativeResize(used+1,1);
      // Update measurement vectors
      OFP::KalmanFilterDynamic<T, 3>::yk = z/2 + dm*d; // New measurement vector

      OFP::KalmanFilterDynamic<T, 3>::ykest = OFP::KalmanFilterDynamic<T, 3>::C*OFP::KalmanFilterDynamic<T, 3>::xHat; // (15) from paper
      // Compute R matrix
      OFP::KalmanFilterDynamic<T, 3>::R = OFP::KalmanFilterDynamic<T, 3>::R.Zero(RDOA.rows()+1,RDOA.rows()+1);
      for(int i=0;i<used;i++) {
        OFP::KalmanFilterDynamic<T, 3>::R(i, i) = 0.5*pow(2*rr_cov,2) + pow(d(i) + dm,2)*2*rr_cov;
        for(int j=0;j<used;j++) {
          if(j!=i) {
            OFP::KalmanFilterDynamic<T, 3>::R(i,j) = 0.5*pow(rr_cov,2) + (d(i)*d(j) + dm*(d(i)+ d(j)) + dm*dm)*rr_cov;
            OFP::KalmanFilterDynamic<T, 3>::R(j,i) = 0.5*pow(rr_cov,2) + (d(j)*d(i) + dm*(d(j)+ d(i)) + dm*dm)*rr_cov;
          }
        }
      }
      OFP::KalmanFilterDynamic<T, 3>::R.bottomRightCorner(1, 1) << rz_cov;

      // For debugging
      //std::cout << "PHat" << std::endl << OFP::KalmanFilterDynamic<T, 3>::PHat << std::endl;
      //std::cout << "C" << std::endl << OFP::KalmanFilterDynamic<T, 3>::C << std::endl;
      //std::cout << "R" << std::endl << OFP::KalmanFilterDynamic<T, 3>::R << std::endl;
      //std::cout << "yk" << std::endl << OFP::KalmanFilterDynamic<T, 3>::yk << std::endl;
      //std::cout << "ykest" << std::endl << OFP::KalmanFilterDynamic<T, 3>::ykest << std::endl;
      return true;
    }
  }
}