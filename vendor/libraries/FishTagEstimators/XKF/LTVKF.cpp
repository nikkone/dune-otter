#include "LTVKF.hpp"
#include <iostream>
namespace FishTagEstimators
{
  namespace XKF
  {
/*    template <class T>
    LTVKF<T>::LTVKF() {
    }*/
template <class T>
bool LTVKF<T>::constructCandR(TagBuffer *tagBuffer, Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations, T m_dr)
    {
     
//std::cout << "RDOA.rows()" << std::endl << RDOA.rows() << std::endl;
const uint minRDOAS=1;
      if(RDOA.rows() < minRDOAS) {
        return false;
      }
      std::cout << "LTVKF RDOA.rows()" << std::endl << RDOA.rows() << std::endl;
      //std::cout << "Enter LTVKF::constructCandR" << std::endl;
      // Resize matrices to maximum probable
      OFP::KalmanFilterDynamic<T, 3>::C.resize(RDOA.rows()+1,OFP::KalmanFilterDynamic<T, 3>::nx);
      Eigen::Matrix<T, Eigen::Dynamic,1> d; // nja in paper
      Eigen::Matrix<T, Eigen::Dynamic,1> z; // y in paper
      d.resize(RDOA.rows()+1,1);
      z.resize(RDOA.rows()+1,1);
      uint32_t referenceReceiver = RDOAcombinations.front().second;
      Eigen::Matrix<T, 3, 1> referenceReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[referenceReceiver]->rbegin())->N, (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->E,(tagBuffer->tagBuffer[referenceReceiver]->rbegin())->D);

      // Compute C matrix
        uint8_t used = 0,combination = 0;
        for(std::vector<std::pair<uint32_t, uint32_t>>::iterator it = RDOAcombinations.begin(); it != RDOAcombinations.end(); it++) {
          if(it->second == referenceReceiver) {
            std::cout << "Combination: " << combination << std::endl;
            Eigen::Matrix<T, 3, 1> currentReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->first]->rbegin())->N, (tagBuffer->tagBuffer[it->first]->rbegin())->E,(tagBuffer->tagBuffer[it->first]->rbegin())->D);
            
            OFP::KalmanFilterDynamic<T, 3>::C.row(used) << -(currentReceiverNED - referenceReceiverNED).transpose();

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
        if(used < minRDOAS) {
            return false;
        }
        if(used !=RDOA.rows()) {
          // TODO: Resize C, z and d
          return false;
        }
        // Add depth measurement after RDOA is added
        std::cout << "Adding" << std::endl;
        OFP::KalmanFilterDynamic<T, 3>::C.row(used) << 0, 0, 0.5;
        z.row(used) << (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->trans_data*tagBuffer->getDepthConversionCoefficient(); // Depth measurement
        d.row(used) << 0;

        // Resize matrices and vectors according to used
        OFP::KalmanFilterDynamic<T, 3>::R.resize(used+1,used+1);
        OFP::KalmanFilterDynamic<T, 3>::yk.resize(used+1,1);
        OFP::KalmanFilterDynamic<T, 3>::ykest.resize(used+1,1);
        OFP::KalmanFilterDynamic<T, 3>::innov.resize(used+1,1);
        
        // Compute R matrix
        OFP::KalmanFilterDynamic<T, 3>::R = OFP::KalmanFilterDynamic<T, 3>::R.Zero(RDOA.rows()+1,RDOA.rows()+1);
        for(int i=0;i<used;i++) {
          OFP::KalmanFilterDynamic<T, 3>::R(i, i) = 0.5*pow(2*rr_cov,2) + (d(i) + m_dr)*(d(i) + m_dr)*2*rr_cov;
          for(int j=0;j<used;j++) {
            if(j!=i) {
              OFP::KalmanFilterDynamic<T, 3>::R(i,j) = 0.5*pow(rr_cov,2) + (d(i)*d(j) + m_dr*(d(i)+ d(j)) + m_dr*m_dr)*rr_cov;
              OFP::KalmanFilterDynamic<T, 3>::R(j,i) = 0.5*pow(rr_cov,2) + (d(j)*d(i) + m_dr*(d(j)+ d(i)) + m_dr*m_dr)*rr_cov;
            }
          }
        }
        OFP::KalmanFilterDynamic<T, 3>::R(used,used) = rz_cov; // Depth covariance

        //std::cout << "Estupdate" << std::endl;
        //std::cout << "C\n" << OFP::KalmanFilterDynamic<T, 3>::C << std::endl;
        // Update measurement vectors
        OFP::KalmanFilterDynamic<T, 3>::yk = 0.5*z+m_dr*d; // New measurement vector
        //std::cout << "Estupdate2" << std::endl;
        OFP::KalmanFilterDynamic<T, 3>::ykest = OFP::KalmanFilterDynamic<T, 3>::C*OFP::KalmanFilterDynamic<T, 3>::xHat; // (15) from paper
        //std::cout << "Estupdate3" << std::endl;
//std::cout << "F::C" << std::endl << C << std::endl;
//std::cout << "F::R" << std::endl << R << std::endl;
//std::cout << "F::yk" << std::endl << yk << std::endl;
//std::cout << "F::ykest" << std::endl << ykest << std::endl;
        return true;
    }
  }
}