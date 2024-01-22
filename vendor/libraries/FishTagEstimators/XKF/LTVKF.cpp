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
      if(RDOA.rows() < 2) {
        return false;
      }
      //std::cout << "Enter LTVKF::constructCandR" << std::endl;
      OFP::KalmanFilterDynamic<T, 3>::C.resize(RDOA.rows()+1,OFP::KalmanFilterDynamic<T, 3>::nx);
      OFP::KalmanFilterDynamic<T, 3>::R.resize(RDOA.rows()+1,RDOA.rows()+1);
      OFP::KalmanFilterDynamic<T, 3>::yk.resize(RDOA.rows()+1,1);
      OFP::KalmanFilterDynamic<T, 3>::ykest.resize(RDOA.rows()+1,1);
      OFP::KalmanFilterDynamic<T, 3>::innov.resize(RDOA.rows()+1,1);

     uint32_t referenceReceiver = RDOAcombinations.front().first;
     Eigen::Matrix<T, 3, 1> referenceReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[referenceReceiver]->rbegin())->N, (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->E,(tagBuffer->tagBuffer[referenceReceiver]->rbegin())->D);
     T tagDepth = (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->getS256Depth();

      
      // Compute C and R matrices
      //C.row(RDOA.rows()) = C.row(RDOA.rows()).Zero(1,3); // May need a +1
      //C.bottomRightCorner(1, 1) << 0.5;
      Eigen::Matrix<T, 3,1> z;
        uint8_t used = 0,combination = 0;
        for(std::vector<std::pair<uint32_t, uint32_t>>::iterator it = RDOAcombinations.begin(); it != RDOAcombinations.end(); it++) {
          if(it->first == referenceReceiver) {
               OFP::KalmanFilterDynamic<T, 3>::C.row(used) << -(
               Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D) -
               referenceReceiverNED
               ).transpose();
                z.row(used) << RDOA(combination)*RDOA(combination) -  
                Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D).squaredNorm() + 
                referenceReceiverNED.squaredNorm();
               used++;
            }
            if(used == 2) {
                OFP::KalmanFilterDynamic<T, 3>::C.row(used) << 0, 0, 0.5;
                z.row(used) << tagDepth;
                break;
            }
            combination++;
        }
        if(used < 2) {
            return false;
        }
        Eigen::Matrix<T, 3,1> d; // nja in paper
        d << RDOA(0), RDOA(1), 0;

//std::cout << "rr_cov: " << rr_cov << std::endl;
        OFP::KalmanFilterDynamic<T, 3>::R = OFP::KalmanFilterDynamic<T, 3>::R.Zero(RDOA.rows()+1,RDOA.rows()+1);
        OFP::KalmanFilterDynamic<T, 3>::R(0, 0) = (1/2)*pow(2*rr_cov,2) + (d(0) + m_dr)*(d(0) + m_dr)*2*rr_cov;
        OFP::KalmanFilterDynamic<T, 3>::R(1, 1) = (1/2)*pow(2*rr_cov,2) + (d(1) + m_dr)*(d(1) + m_dr)*2*rr_cov;
        OFP::KalmanFilterDynamic<T, 3>::R(0,1) = 0.5*pow(rr_cov,2) + (d(0)*d(1) + m_dr*(d(0)+ d(1)) + m_dr*m_dr)*rr_cov;
        OFP::KalmanFilterDynamic<T, 3>::R(1,0) = 0.5*pow(rr_cov,2) + (d(0)*d(1) + m_dr*(d(0)+ d(1)) + m_dr*m_dr)*rr_cov;
        OFP::KalmanFilterDynamic<T, 3>::R(2,2) = rz_cov;

        OFP::KalmanFilterDynamic<T, 3>::yk = 0.5*z+m_dr*d;

        OFP::KalmanFilterDynamic<T, 3>::ykest = OFP::KalmanFilterDynamic<T, 3>::C*OFP::KalmanFilterDynamic<T, 3>::xHat; // (15) from paper
//std::cout << "F::C" << std::endl << C << std::endl;
//std::cout << "F::R" << std::endl << R << std::endl;
//std::cout << "F::yk" << std::endl << yk << std::endl;
//std::cout << "F::ykest" << std::endl << ykest << std::endl;
        return true;
    }
  }
}