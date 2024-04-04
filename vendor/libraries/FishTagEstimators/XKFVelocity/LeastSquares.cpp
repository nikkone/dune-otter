#include "LeastSquares.hpp"
#include <Eigen/Dense>
#include <iostream>
namespace FishTagEstimators
{
  namespace XKFVelocity
  {
    //! method for calculating the pseudo-Inverse as recommended by Eigen developers
    //! Source: https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
    template<typename _Matrix_Type_>
    _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
    {
      Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
            // For a non-square matrix
            // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
      double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
      return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }

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
    bool LeastSquares<T>::update(const TagBuffer *tagBuffer, const Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, const std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations)
    {
      if(RDOA.rows() < 2) {
        return false;
      }

      // Construct Least squares and measurement matrices
      Eigen::Matrix<T, 3,3> Cyq;
      Eigen::Matrix<T, Eigen::Dynamic,1> l; // nja in paper
      l.resize(RDOA.rows()+1,1);

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
          Cyq.row(used) << -(
          currentReceiverNED -
          referenceReceiverNED).transpose();
          // Note: SquaredNorm(x)=||x||^2
          z.row(used) << RDOA(combination)*RDOA(combination) -
          currentReceiverNED.squaredNorm() + 
          referenceReceiverNED.squaredNorm();

          l.row(used) << RDOA(combination);
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
      Cyq.row(used) = Cyq.row(used).Zero(1,3);
      Cyq.bottomRightCorner(1, 1) << 0.5;
      z.row(used) << (tagBuffer->tagBuffer.at(referenceReceiver)->rbegin())->trans_data*tagBuffer->getDepthConversionCoefficient(); // Depth measurement
      l.row(used) << 0;

      // Compute solution
      Eigen::Matrix<T, 3,3> invCyq = pseudoInverse(Cyq);
      Eigen::Matrix<T, 3,1> c = invCyq*l; // nja overline
      Eigen::Matrix<T, 3,1> w = 0.5*invCyq*z; // Y overline

      // quadratic equation coefficients for computation of d3 = m_dr, p = p_r
      T ctc = c.transpose()*c;
      T aa = 1 - ctc; // 1 - c'c
      T bb = 2*(referenceReceiverNED.transpose()*c - w.transpose()*c)(0); // 2(p'c - w'c)
      T cc = -1*(referenceReceiverNED - w).transpose()*(referenceReceiverNED - w); //Equivalent to 2p'w - w'w - p'p

      //std::cout << "invCyq:" << std::endl << invCyq << std::endl;
      // Compute solution for quadratic term
      T R1, R2;
      if(ctc == 1) { // Equivalent to aa=0
        R1 = -cc/bb;
        //std::cout << "CTC=1" << std::endl; 
        // Unique solution
      } else {
        if((bb*bb - 4*aa*cc) <= 0.0) {
          R1 = -bb/(2*aa);
          //std::cout << "(bb*bb - 4*aa*cc) <= 0.0" << std::endl;
          //std::cout << "a, b, c: "<< aa << ", " << bb << ", " << cc << std::endl;
          // Unique solution
        } else {
          T s = sqrt(bb*bb - 4*aa*cc);
          R1 = (-bb + s)/(2*aa);
          R2 = (-bb - s)/(2*aa);
          //std::cout << "Resolve ambiguity R1: " << R1 << ", R2: "<< R2 << std::endl; 
          R1 = resolveRAmbiguity(R1, R2);
        }
      }
      if((R1 > 0.0) && (R1 < maxDm)) {
        xHat = (R1*c + w);
        dm = R1;
        return true;
      }
      //std::cout << "Could not dm. R1: " << R1 << std::endl;
      //std::cout << "a, b, c: "<< aa << ", " << bb << ", " << cc << std::endl;
      return false;
    }

    template <class T>
    T LeastSquares<T>::resolveRAmbiguity(T R1, T R2)
    {
      T R_temp;
      if((R1 > 0.0) && (R1 < maxDm)) {
        if((R2 > 0.0) && (R2 < maxDm)) { // Both valid, choose one of them 
          R_temp = R1; // TODO: Find a way to choose
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