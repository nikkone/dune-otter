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
std::cout << "Entered FISHTAG::LeastSquares::update" <<  std::endl;
for(auto combination : RDOAcombinations) {
  std::cout << combination.first <<" - "<< combination.second << std::endl;
}

      uint32_t referenceReceiver = RDOAcombinations.front().second;
      Eigen::Matrix<T, 3, 1> referenceReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[referenceReceiver]->rbegin())->N, (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->E,(tagBuffer->tagBuffer[referenceReceiver]->rbegin())->D);
      // Construct Least squares and measurement matrices
      Eigen::Matrix<T, 3,1> l; // nja in paper
      l << RDOA(0), RDOA(1), 0;

      Eigen::Matrix<T, 3,3> Cyq;
      Eigen::Matrix<T, 3,1> z; // Y from paper ( applied Eq (8) in paper)
      uint8_t used = 0,combination = 0;
      for(std::vector<std::pair<uint32_t, uint32_t>>::iterator it = RDOAcombinations.begin(); it != RDOAcombinations.end(); it++) {
        if(it->second == referenceReceiver) {
          Cyq.row(used) << -(
          Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->first]->rbegin())->N, (tagBuffer->tagBuffer[it->first]->rbegin())->E,(tagBuffer->tagBuffer[it->first]->rbegin())->D) -
          referenceReceiverNED).transpose();
          // Note: SquaredNorm(x)=||x||^2
          z.row(used) << RDOA(combination)*RDOA(combination) -
          Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer[it->first]->rbegin())->N, (tagBuffer->tagBuffer[it->first]->rbegin())->E,(tagBuffer->tagBuffer[it->first]->rbegin())->D).squaredNorm() + 
          referenceReceiverNED.squaredNorm();
          used++;
        }
        if(used == 2) {
          Cyq.row(used) << 0, 0, 0.5;
          z.row(used) << (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->trans_data*tagBuffer->getDepthConversionCoefficient(); // Tag depth
          break;
        }
        combination++;
      }
      if(used < 2) {
          std::cout << "Too few used" <<  std::endl;
          return false;
      }

      Eigen::Matrix<T, 3,3> invCyq = (Cyq.transpose()*Cyq).inverse()*Cyq.transpose();
      Eigen::Matrix<T, 3,1> c = invCyq*l; // nja overline
      Eigen::Matrix<T, 3,1> w = 0.5*invCyq*z; // Y overline
      // temp variables
      T ctc = c.transpose()*c;//c(0,0)*c(0,0) + c(1,0)*c(1,0) + c(2,0)*c(2,0);
      T aa = 1 - ctc;
      T bb = 2*(referenceReceiverNED.transpose()*c-w.transpose()*c)(0);
      T cc = -1*(referenceReceiverNED - w).squaredNorm();

      // Compute solution for quadratic term
      T R1, R2;
      if(ctc == 1) { // Equivalent to aa=0
        R1 = -cc/bb;
        std::cout << "CTC=1" << std::endl; 
        // Unique solution
      } else {
        if((bb*bb - 4*aa*cc) <= 0.0) {
          R1 = -bb/(2*aa);
          std::cout << "(bb*bb - 4*aa*cc) <= 0.0" << std::endl; 
          // Unique solution
        } else {
          T s = sqrt(bb*bb - 4*aa*cc);
          R1 = (-bb + s)/(2*aa);
          R2 = (-bb - s)/(2*aa);
          std::cout << "Resolve ambiguity R1: " << R1 << ", R2: "<< R2 << std::endl; 
          R1 = resolveRAmbiguity(R1, R2);
        }
      }
      if((R1 > 0.0) && (R1 < maxDm)) {
        xHat = (R1*c + w);
        dm = R1;
        return true;
      }
      std::cout << "Could not dm. R1: " << R1 << std::endl;
      std::cout << "a, b, c: "<< aa << ", " << bb << ", " << cc << std::endl;
      return false;
    }

    template <class T>
    T LeastSquares<T>::resolveRAmbiguity(T R1, T R2)
    {
      T R_temp;
      if((R1 > 0.0) && (R1 < maxDm)) {
        if((R2 > 0.0) && (R2 < maxDm)) { // Both valid, choose one of them 
          R_temp = R1; // TODO: some trick here will help
        } else { // Only R1 valid
          R_temp = R1;
        }
      } else {
        if((R2 > 0.0) && (R2 < maxDm)) {// Only R2 valid
          R_temp = R2;
        } else { // Both negative, none valid
          R_temp = 0;
        }
      }
      return R_temp;
    }
  }
}