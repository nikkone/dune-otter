#include "MultipleReceiversXKF.hpp"
#include <iostream>
#include <iomanip>      // std::setprecision
#include <vector>
namespace FishTagEstimators
{
  template <class T>
  void MultipleReceiverXKF<T>::initialize(const Eigen::Matrix<T, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<T, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<T, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<T, c_states, 1> &x0_inn)
  {
    name = "MultiReceiverXKF";

    //! TODO: Set D matrix, and use it for predict?
    //! Use Phat to descide if additional measurment methods can be used
    //! 
    //! 
    //! 
    //! 
    Estimator::initialize(A_inn, Q_inn, P0_inn, x0_inn);
    xkf.initialize(A_inn, Q_inn, P0_inn, x0_inn);
    //std::cout << "XKF initialize" << std::endl;
  }
  
  template <class T>
  void MultipleReceiverXKF<T>::predict() {
    xkf.predict();
  }

  template <class T>
  std::tuple<T, T, T> MultipleReceiverXKF<T>::getEstimate() const{
    /*
    std::cout << "Logging from multirecvXKF: Xhat" << std::endl << xkf.stage3.xHat << std::endl;
     std::cout << "Xhat2" << std::endl << xkf.stage2.xHat << std::endl;
     std::cout << "Xhat1" << std::endl << xkf.stage2.xHat << std::endl;*/
    return {xkf.stage3.xHat(0),xkf.stage3.xHat(1),xkf.stage3.xHat(2)};
    //return {xkf.stage2.xHat(0),xkf.stage2.xHat(1),xkf.stage2.xHat(2)};
    //return {xkf.stage1.xHat(0),xkf.stage1.xHat(1),xkf.stage1.xHat(2)};

    //return {0.0,0.0,0.0};
  }

  template <class T>
  void MultipleReceiverXKF<T>::print(std::ostream& os) const {
    os << "XKF Stage 3 A" << std::endl << xkf.stage3.A << std::endl;
    os << "XKF Stage 3 C" << std::endl << xkf.stage3.C << std::endl;
    os << "XKF Stage 3 PHat" << std::endl << xkf.stage3.PHat << std::endl;
    os << "XKF Stage 3 xHat" << std::endl << xkf.stage3.xHat << std::endl;
    os << "XKF Stage 3 R" << std::endl << xkf.stage3.R << std::endl;
    os << "XKF Stage 3 Q" << std::endl << xkf.stage3.Q << std::endl;
    os << "XKF Stage 3 K" << std::endl << xkf.stage3.K << std::endl;
    os << "XKF Stage 3 ykest" << std::endl << xkf.stage3.ykest << std::endl;
    os << "XKF Stage 3 yk" << std::endl << xkf.stage3.yk << std::endl;
    os << "XKF Stage 3 innov" << std::endl << xkf.stage3.innov << std::endl;
  }

  template <class T>
  void MultipleReceiverXKF<T>::setTDOACovariance(T TDOACovariance) {
    rr_cov = TDOACovariance;
    xkf.setDiagonalCovarianceR(TDOACovariance);
  }

  template <class T>
  void MultipleReceiverXKF<T>::setDepthCovariance(T depthCovariance) {
    rz_cov = depthCovariance;
    xkf.setVarianceRZ(depthCovariance);
  }

  template <class T>
  bool MultipleReceiverXKF<T>::update(TagBuffer *tagBuffer) {
    //std::cout << "Entering MultipleReceiverXKF<T>::update" << std::endl;
    if(tagBuffer->tagBuffer.size() <2)
      return false;

    Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA(0 ,1);
    std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations;
    tagBool_t used; // Could have been made standard vector, only need receiver address. But if we move toestimator?
    tagBool_t tempUsed; // Could have been made standard vector, only need receiver address. But if we move toestimator?
    unsigned baselines = 0;
    uint8_t status = 0;
    std::vector<std::pair<double, double>> tempLatestReceiverPositionsUsed;
    for(auto refReceiver : tagBuffer->tagBuffer) {
      if(!unprocessedData[refReceiver.first]) {
        continue; // If reference receiver does not contain new data, jumpt to next
      }
      uint32_t referenceReceiver = refReceiver.first;
      long int referenceReceiverToA = (long int)refReceiver.second->rbegin()->unix_timestamp*1000 + (long int)refReceiver.second->rbegin()->millis;
      for(auto receiver : tagBuffer->tagBuffer) {
        if(unprocessedData[receiver.first]) {
          if(referenceReceiver != receiver.first) {
            long int tempTDOA_ms = (long int)receiver.second->rbegin()->unix_timestamp*1000 + (long int)receiver.second->rbegin()->millis - referenceReceiverToA;
            if(timeShiftCorrect(tempTDOA_ms)) {
              RDOAcombinations.push_back(std::pair<uint32_t, uint32_t>(receiver.first, referenceReceiver));
              RDOA.conservativeResize(RDOA.rows()+1,1);
              RDOA(RDOA.rows()-1,0) = c_speed*tempTDOA_ms/1000;
              tempUsed[referenceReceiver] = true;
              tempUsed[receiver.first] = true;
              baselines++;
            }
          }
        }
      }
      if(baselines > 1) { // Do not process data/update filter if too few baselines available
        // Run filter update, and if sucessfull, set unprocessedData to false for used data receivers
        status = xkf.update(tagBuffer, RDOA, RDOAcombinations);
        switch(status) {
          case 4: // Only stage 3 update
          {
            std::cout << "Only stage3 update" << std::endl;
            used = tempUsed;
            for(auto it : tempUsed) {
              tempLatestReceiverPositionsUsed.push_back(std::pair<T,T>((tagBuffer->tagBuffer[it.first]->rbegin())->N, (tagBuffer->tagBuffer[it.first]->rbegin())->E));
            }
          } 
          break;
          case 6: // Only stage 2 and 3 update
          {
            std::cout << "Only stage2 and stage3 update" << std::endl;
            used = tempUsed;
            for(auto it : tempUsed) {
              tempLatestReceiverPositionsUsed.push_back(std::pair<T,T>((tagBuffer->tagBuffer[it.first]->rbegin())->N, (tagBuffer->tagBuffer[it.first]->rbegin())->E));
            }
          } 
          break;
          case 7: // All steps updated
          {
            std::cout << std::endl << "All steps updated" << std::endl;
            latestReceiverPositionsUsed.clear();
            for(auto it : tempUsed) {
              unprocessedData[it.first] = false;
              tempLatestReceiverPositionsUsed.push_back(std::pair<T,T>((tagBuffer->tagBuffer[it.first]->rbegin())->N, (tagBuffer->tagBuffer[it.first]->rbegin())->E));
            }
            std::cout << std::endl << "Status: " << (int)status << std::endl;
            //return true;
          }
          break;
          default:
          // Stage 1 and 2 only
          // No update
          break;
        }
        RDOAcombinations.clear();
        RDOA.resize(0 ,1);
      } else {
        break;
      }
      std::cout << std::endl << "Retrying with other ref." << std::endl;
    }
    std::cout << std::endl << "Status: " << (int)status << std::endl;
    if(status) {
      for(tagBool_t::iterator it = used.begin();it != used.end();it++) {
        unprocessedData[it->first] = false;
      }
      latestReceiverPositionsUsed = tempLatestReceiverPositionsUsed;
      return true;
    }

    return false;
  }


template <class T>
  bool MultipleReceiverXKF<T>::checkTime(T transmissionFirstTime, T currentTime) const {
    std::cout << std::setprecision(11) << std::endl << "Current, first: "<< transmissionFirstTime << ", " << currentTime << std::endl;
    if(currentTime - transmissionFirstTime > 1.5) {
      return true;
    } else {
      return false;
    }
  }

  template <class T>
  void MultipleReceiverXKF<T>::estimateToStream(std::ostream& os) const {
    os << xkf.stage3.xHat(0) << "," << xkf.stage3.xHat(1) << "," <<xkf.stage3.xHat(2);
  }
}