#include "LeastSquares.hpp"
#include <Eigen/Dense>
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
    LeastSquares<T>::LeastSquares() : maxDm(550), B_fit(49.807), k_fit(4.9147), a_fit(0.015309) {
        xHat <<0,0,0;
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
      
      long int minToA_ms = (long int)(tagBuffer->tagBuffer.at(referenceReceiver)->rbegin())->unix_timestamp*1000 + (long int)(tagBuffer->tagBuffer.at(referenceReceiver)->rbegin())->millis;
      Eigen::Matrix<T, 3, 1> minToAReceiverNED = referenceReceiverNED;
      
      uint8_t used = 0, combination = 0;
      for(auto it : RDOAcombinations) {
        if(it.second == referenceReceiver) {
          // Current
          Eigen::Matrix<T, 3, 1> currentReceiverNED = Eigen::Matrix<T, 3, 1>((tagBuffer->tagBuffer.at(it.first)->rbegin())->N, (tagBuffer->tagBuffer.at(it.first)->rbegin())->E,(tagBuffer->tagBuffer.at(it.first)->rbegin())->D);
          long int currentToA_ms = (long int)(tagBuffer->tagBuffer.at(it.first)->rbegin())->unix_timestamp*1000 + (long int)(tagBuffer->tagBuffer.at(it.first)->rbegin())->millis;
          if(minToA_ms > currentToA_ms) {
            minToA_ms = currentToA_ms;
            minToAReceiverNED = currentReceiverNED;
          }

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

          used = 0;
          std::vector<TBRFishTag> tagDetections;
          tagDetections.push_back(*(tagBuffer->tagBuffer.at(referenceReceiver)->rbegin()));
          for(auto it : RDOAcombinations) {
            if(it.second == referenceReceiver) {
              tagDetections.push_back(*(tagBuffer->tagBuffer.at(it.first)->rbegin()));
              used++;
            }
            if(used == 2) {
              break;
            }
          }
          //std::cout << "Resolve ambiguity R1: " << R1 << ", R2: "<< R2 << std::endl; 
          R1 = resolveRAmbiguity(R1, R2, tagDetections, c, w);
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
    T LeastSquares<T>::resolveRAmbiguity(T R1, T R2, std::vector<TBRFishTag> &tagDetections, const Eigen::Matrix<T, 3,1> c, const Eigen::Matrix<T, 3,1> &w)
    {
      T R_temp;
      if((R1 > 0.0) && (R1 < maxDm)) {
        if((R2 > 0.0) && (R2 < maxDm)) { // Both valid, choose one of them
          // Option 1: Choose Rx that makes xHat closest to the position of smallest ToA
          /*T d1 = ((R1*c + w) - minToAReceiverNED).squaredNorm();
          T d2 = ((R2*c + w) - minToAReceiverNED).squaredNorm();
          if(d1<d2) {
            R_temp = R1;
          } else {
            R_temp = R2;
          }*/
          
          // Option 2: Choose RX that makes reception order correct according to ToA
          // Calculate position of both solutions
          Eigen::Matrix<T, 3,1> xHat1 = (R1*c + w);
          Eigen::Matrix<T, 3,1> xHat2 = (R2*c + w);
          //Order by ToA ascending
          std::sort(tagDetections.begin(), tagDetections.end(), TBRFishTag::compareByTOA);
        
          // Check if receive order is correct for R1 and calculate squared error from SNR model
          T dist = 0;
          bool R1valid = true;
          T SNRerror1 = 0;
          for(auto it: tagDetections) {
            T d1 = (xHat1 - Eigen::Matrix<T, 3, 1>(it.N, it.E, it.D)).norm();
            T SNRest = B_fit - k_fit*std::log10(d1) - a_fit*d1;
            SNRerror1 = SNRerror1 + std::pow(it.snr - SNRest, 2);
            std::cout << it.serial_no << " - " << it.millis << " - " << d1 << " - " << int(it.snr) << " - " << SNRest << " - " << SNRerror1 << std::endl;
            if(d1 < dist) {
              R1valid = false;
              break;
            } else {
              dist = d1;
            }
          }
          
          // Check if receive order is correct for R2 and calculate squared error from SNR model
          dist = 0;
          bool R2valid = true;
          T SNRerror2 = 0;
          for(auto it: tagDetections) {
            T d1 = (xHat2 - Eigen::Matrix<T, 3, 1>(it.N, it.E, it.D)).norm();
            T SNRest = B_fit - k_fit*std::log10(d1) - a_fit*d1;
            SNRerror2 = SNRerror2 + std::pow(it.snr - SNRest, 2);
            std::cout << it.serial_no << " - " << it.millis << " - " << d1 << " - " << int(it.snr) << " - " << SNRest << " - " << SNRerror2 << std::endl;
            if(d1 < dist) {
              R2valid = false;
              break;
            } else {
              dist = d1;
            }
          }
          // Select option according to TOA receive order first, and if still tie, use SNR model
          if(R1valid || R2valid){
            if(R1valid && R2valid) {
              if(SNRerror1 < SNRerror2) {
                std::cout << "Both TOA valid, R1 least SNR error. SNRerr1: " <<SNRerror1 << ", SNRerr2: " << SNRerror2 << std::endl;
                R_temp = R1;
              } else {
                std::cout << "Both TOA valid, R2 least SNR error. SNRerr1: " << SNRerror1 << ", SNRerr2: " << SNRerror2 << std::endl;
                R_temp = R2;
              }
            } else if(R1valid) {
              std::cout << "Only R1 TOA valid" << std::endl;
              R_temp = R1;
            } else {
              std::cout << "Only R2 TOA valid" << std::endl;
              R_temp = R2;
            }
          } else {
            if(SNRerror1 < SNRerror2) {
              std::cout << "None TOA valid, R1 least SNR error. SNRerr1: " <<SNRerror1 << ", SNRerr2: " << SNRerror2 << std::endl;
              R_temp = R1;
            } else {
              std::cout << "None TOA valid, R2 least SNR error. SNRerr1: " << SNRerror1 << ", SNRerr2: " << SNRerror2 << std::endl;
              R_temp = R2;
            }
          }
          
         // Option 3: Choose R1/R2 always
          //R_temp = R1;
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
