#include "MultipleReceiversXKF.hpp"
#include <iostream>
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
    //! 
    Estimator::initialize(A_inn, Q_inn, P0_inn, x0_inn);
    xkf.initialize(A_inn, Q_inn, P0_inn, x0_inn);
  }
  
  template <class T>
  void MultipleReceiverXKF<T>::predict() {
    xkf.predict();
  }

  template <class T>
  std::tuple<T, T, T> MultipleReceiverXKF<T>::getEstimate() {
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
    Eigen::Matrix<T, Eigen::Dynamic, 1> depth(tagBuffer->tagBuffer.size(),1);
    tagBool_t used; // Could have been made standard vector, only need receiver address. But if we move toestimator?

    unsigned baselines = 0;
    std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations;
    for (tagBufferMap_t::const_iterator outerreceiver = tagBuffer->tagBuffer.begin(); outerreceiver != tagBuffer->tagBuffer.end(); outerreceiver++) {
      for (tagBufferMap_t::const_iterator receiver = std::next(outerreceiver); receiver != tagBuffer->tagBuffer.end(); receiver++) {
        if( unprocessedData[outerreceiver->first] || unprocessedData[receiver->first] ) {
          if( used.find(outerreceiver->first) == used.end() || used.find(receiver->first) == used.end()) {
            long int tempTDOA_ms = ((long int)receiver->second->rbegin()->unix_timestamp - outerreceiver->second->rbegin()->unix_timestamp)*1000 + ((int)receiver->second->rbegin()->millis - outerreceiver->second->rbegin()->millis);
            if(timeShiftCorrect(tempTDOA_ms)) {
              RDOAcombinations .push_back(std::pair<uint32_t, uint32_t>(outerreceiver->first, receiver->first));
              RDOA.conservativeResize(RDOA.rows()+1,1);
              RDOA(RDOA.rows()-1,0) = c_speed*tempTDOA_ms/1000;
              depth.row(baselines) << (outerreceiver->second->rbegin()->trans_data + receiver->second->rbegin()->trans_data)/2;
              used[outerreceiver->first] = false;
              used[receiver->first] = false;
              baselines++;
            }
          }
        }
      }
    }

    if(baselines <2) { // Todo: Make change to one possible
      return false; // Do not process data/update filter if no baselines available
    }

    // Run filter update, and if sucessfull, set unprocessedData to false for used data receivers
    if(xkf.update(tagBuffer, RDOA, RDOAcombinations)) {
      for(tagBool_t::iterator it = used.begin();it != used.end();it++) {
        unprocessedData[it->first] = false;
      }
      return true;
    }
    return false;
  }

template <class T>
  bool MultipleReceiverXKF<T>::checkTime(T transmissionFirstTime, T currentTime) const {
    if(currentTime - transmissionFirstTime > 1.5) {
      return true;
    } else {
      return false;
    }
  }
}