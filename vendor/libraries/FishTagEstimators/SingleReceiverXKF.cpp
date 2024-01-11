#include "SingleReceiverXKF.hpp"
#include "DUNETagBuffer.hpp"
namespace FishTagEstimators
{


  void SingleReceiverXKF::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    SingleReceiverBase::initialize(A_inn, Q_inn, P0_inn, x0_inn);
    xkf.initialize(A_inn, Q_inn, P0_inn, x0_inn);
    name = "SingleReceiverXKF";
  }

  void SingleReceiverXKF::predict() {
    if(!isActive()) {
      xkf.predict();
    }
  }

  std::tuple<double, double, double> SingleReceiverXKF::getEstimate() {
    return {xkf.stage3.xHat(0),xkf.stage3.xHat(1),xkf.stage3.xHat(2)};
    //return {xkf.stage2.xHat(0),xkf.stage2.xHat(1),xkf.stage2.xHat(2)};
    //return {xkf.stage1.xHat(0),xkf.stage1.xHat(1),xkf.stage1.xHat(2)};
  }

  void SingleReceiverXKF::print(std::ostream& os) const {
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


  void SingleReceiverXKF::setTDOACovariance(double TDOACovariance) {
    rr_cov = TDOACovariance;
    xkf.setDiagonalCovarianceR(TDOACovariance);
  }

  void SingleReceiverXKF::setDepthCovariance(double depthCovariance) {
    rz_cov = depthCovariance;
    xkf.setVarianceRZ(depthCovariance);
  }

  bool SingleReceiverXKF::update(TagBuffer *tagBuffer) {
    SingleReceiverBase::update(tagBuffer);
    if(tag_period <= 0) {
      return false;
    }
    if(tagBuffer->tagBuffer.find(receiver) ==tagBuffer->tagBuffer.end()) {
      std::cout << "Did not find receiver: " << receiver << std::endl;
      return false;
    }
    
    const tagBuffer_t *receiverBuffer = tagBuffer->tagBuffer.find(receiver)->second;

    FishTagEstimators::TagBuffer tempTagBuffer(tagBuffer->tag_id, 1);
    Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA(0 ,1);
    std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations;
    tagBool_t used; // Could have been made standard vector, only need receiver address. But if we move toestimator?
    /* //Only for debug
    for(auto it : *receiverBuffer) {
      std::cout << "test:" <<  it.unix_timestamp << std::endl;
      tempTagBuffer.tagBuffer[it.serial_no+i] = new tagBuffer_t(2);
      tempTagBuffer.tagBuffer[it.serial_no+i]->push_back(it);
      i++;
    }*/


    // Create unix timestamp in milliseconds for the most recent measurement
    long int measurement_ms = (long int)receiverBuffer->rbegin()->unix_timestamp*1000 + receiverBuffer->rbegin()->millis;

    tempTagBuffer.tagBuffer[receiver] = new tagBuffer_t(1);
    tempTagBuffer.tagBuffer[receiver]->push_back(*(receiverBuffer->rbegin()));
    // Depth reading from the current tag
    if(depthConversion < 0.01) {
      tempTagBuffer.tagBuffer[receiver]->begin()->trans_data = std::abs(std::round(receiver_depth/0.392));
    }
    int receiverAdder=1;

    // Check the buffer of older tag detections from the second newest to the oldest.
    // Only combine if a multiple of the period is found within a given threashold/jitter.
    for(tagBuffer_t::const_reverse_iterator i=receiverBuffer->rbegin()+1; i != receiverBuffer->rend();i++) {

      // Time difference of arrival without correcting for period
      long int td = measurement_ms - (long int)i->unix_timestamp*1000 - i->millis;

      // Calculate closest multiple of period between the new measurement and the buffered detection
      //! TODO: For non-regular tag_period, will not work. Need to find period for each delta. 
      long int closestMultipleOfPeriod_ms = tag_period*1000*std::round(td/(tag_period*1000));

      // Period corrigated time difference of arrival
      long int tempTDOA_ms = td - closestMultipleOfPeriod_ms;
      std::cout << "measurement_ms1: " << measurement_ms << std::endl;
      std::cout << "measurement_ms2: " << (long int)i->unix_timestamp*1000 - i->millis << std::endl;
      std::cout << "td: " << td << std::endl;
      std::cout << "closestMultipleOfPeriod_ms: " << closestMultipleOfPeriod_ms << std::endl;
      std::cout << "tempTDOA_ms: " << tempTDOA_ms << std::endl;
      //std::cout << ": " << << std::endl;
      // Check if buffered detection satisfies conditions for use in estimator
      if(std::abs(tempTDOA_ms) < max_jitter*1000 && std::abs(tempTDOA_ms) > 0) {   
        // Add to temporary tagBuffer
        tempTagBuffer.tagBuffer[receiver+receiverAdder] = new tagBuffer_t(1);
        tempTagBuffer.tagBuffer[receiver+receiverAdder]->push_back(*i);

        RDOAcombinations.push_back(std::pair<uint32_t, uint32_t>(receiverBuffer->rbegin()->serial_no, receiver+receiverAdder));
        RDOA.conservativeResize(RDOA.rows()+1,1);
        RDOA(RDOA.rows()-1,0) = -1*c_speed*tempTDOA_ms/1000;
        // Depth reading from the current tag
        if(depthConversion < 0.01) {
          tempTagBuffer.tagBuffer[receiver+receiverAdder]->begin()->trans_data = std::abs(std::round(receiver_depth/0.392));
          //std::cout << "Depth" << tempTagBuffer.tagBuffer[receiver+receiverAdder]->begin()->trans_data << std::endl;
        }
        receiverAdder++;
        if(receiverAdder >2) {
          break; 
        }
      }
    }
/* // For debugging
    for (tagBufferMap_t::iterator it = tempTagBuffer.tagBuffer.begin(); it != tempTagBuffer.tagBuffer.end(); it++) {
      std::cout << "Receiver:" <<  it->first << std::endl;
      std::cout << "Timestamp:" <<  it->second->begin()->unix_timestamp << std::endl;
    }
*/
    if(receiverAdder <2) { // Todo: Make change to one possible
      return false; // Do not process data/update filter if no baselines available
    }

    std::cout << "RDOA" << std::endl << RDOA << std::endl;
    std::cout << "RDOAcombinations" << std::endl;
    for(auto const &it : RDOAcombinations) {
      std::cout << it.first << " - " << it.second << std::endl;
    }
    // Run filter update, and if sucessfull, set unprocessedData to false for used data receivers
    if(xkf.update(&tempTagBuffer, RDOA, RDOAcombinations)) {
      latestTimestamp = ((uint64_t)tagBuffer->tagBuffer.begin()->second->rbegin()->unix_timestamp)*1000 + tagBuffer->tagBuffer.begin()->second->rbegin()->millis;
      activateEstimator();
      return true;
    }

    return false;
  }
  bool SingleReceiverXKF::checkTime(double transmissionFirstTime, double currentTime) const {
    if(currentTime - transmissionFirstTime > 1.5) {
      return true;
    } else {
      return false;
    }
  }
}