#include "LeastSquares.hpp"
#include <Eigen/Dense>
#include <iostream>
namespace FishTagEstimators
{
  namespace XKF
  {
    template <class T>
    LeastSquares<T>::LeastSquares() {
        xHat <<0,0,0;
        maxDm = 700; //Default
    }

    template <class T>
    void LeastSquares<T>::setMaxDm(T in_maxDm) {
        maxDm = in_maxDm;
    }

    template <class T>
    bool LeastSquares<T>::update(TagBuffer *tagBuffer, Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations)
    {
      if(RDOA.rows() < 2) {
        return false;
      }
//std::cout << "Entered FISHTAG::LeastSquares::update" <<  std::endl;
      uint32_t referenceReceiver = RDOAcombinations.front().first;
      Eigen::Matrix<T, 3, 1> referenceReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[referenceReceiver]->rbegin())->N, (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->E,(tagBuffer->tagBuffer[referenceReceiver]->rbegin())->D);
      // >> Construct Least squares and measurement matrices
      Eigen::Matrix<T, 3,1> l; // nja in paper
      l << RDOA(0), RDOA(1), 0;

      Eigen::Matrix<T, 3,3> Czq;
      Eigen::Matrix<T, 3,1> z; // Y from paper ( applied Eq (8) in paper)
      uint8_t used = 0,combination = 0;
      for(std::vector<std::pair<uint32_t, uint32_t>>::iterator it = RDOAcombinations.begin(); it != RDOAcombinations.end(); it++) {
        if(it->first == referenceReceiver) {
          Czq.row(used) << -(
          Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D) -
          referenceReceiverNED).transpose();
          z.row(used) << RDOA(combination)*RDOA(combination) -
          Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D).squaredNorm() + 
          referenceReceiverNED.squaredNorm();
          used++;
        }
        if(used == 2) {
          Czq.row(used) << 0, 0, 1;
          z.row(used) << (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->getS256Depth(); // Tag depth
          break;
        }
        combination++;
      }
      if(used < 2) {
          return false;
      }

      Eigen::Matrix<T, 3,3> invCzq = (Czq.transpose()*Czq).inverse()*Czq.transpose();
      Eigen::Matrix<T, 3,1> c = invCzq*l; // nja overline
      Eigen::Matrix<T, 3,1> w = 0.5*invCzq*z; // Y overline
      // temp variables
      T ctc = c(0,0)*c(0,0) + c(1,0)*c(1,0) + c(2,0)*c(2,0);
      T aa = 1 - c.transpose()*c;
      T bb = 2*(referenceReceiverNED.transpose()*c-w.transpose()*c)(0);
      T cc = -1*(referenceReceiverNED - w).squaredNorm();

      // Compute solution for quadratic term
      T R1, R2;
      if(ctc == 1) {
        R1 = -cc/bb;
        // Unique solution
      } else {
        if((bb*bb - 4*aa*cc) <= 0.0) {
          R1 = -bb/(2*aa);
          // Unique solution
        } else {
          T s = sqrt(bb*bb - 4*aa*cc);
          R1 = (-bb + s)/(2*aa);
          R2 = (-bb - s)/(2*aa);
          R1 = resolveRAmbiguity(R1, R2);
        }
      }
      if((R1 > 0.0) && (R1 < maxDm)) {
        xHat = (R1*c + w);
        dm = R1;
        return true;
      }

      return false;
    }

    template <class T>
    T LeastSquares<T>::resolveRAmbiguity(T R1, T R2)
    {
      T R_temp;
      if((R1 > 0.0) && (R1 < maxDm)) {
        if((R2 > 0.0) && (R2 < maxDm)) { // Cannot resolve ambiguity, both valid, choose one of them 
          R_temp = R1; // TODO: some trick here will help
        } else { // R1 valid
          R_temp = R1;
        }
      } else {
        if((R2 > 0.0) && (R2 < maxDm)) {// R2 valid
          R_temp = R2;
        } else { // Could not resolve ambiguity
          R_temp = 0;
        }
      }
      return R_temp;
    }
  }
}