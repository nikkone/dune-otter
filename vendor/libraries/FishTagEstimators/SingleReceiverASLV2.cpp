#include "SingleReceiverASLV2.hpp"
namespace FishTagEstimators
{
  std::tuple<double, double, double> SingleReceiverASLV2::getEstimate() {
    return {aslv.xHat(0),aslv.xHat(1),aslv.xHat(2)};
  }

  void SingleReceiverASLV2::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    SingleReceiverBase::initialize(A_inn, Q_inn, P0_inn, x0_inn);
    name = "SingleReceiverASLV2";
  }


  bool SingleReceiverASLV2::update(TagBuffer *tagBuffer) {
    SingleReceiverBase::update(tagBuffer);
    if(tag_period <= 0) {
      return false;
    }
    if(tagBuffer->tagBuffer.find(receiver) == tagBuffer->tagBuffer.end()) {
      std::cout << "Did not find receiver: " << receiver << std::endl;
      return false;
    }
    
    const tagBuffer_t *receiverBuffer = tagBuffer->tagBuffer.find(receiver)->second;

    FishTagEstimators::TagBuffer tempTagBuffer(tagBuffer->tag_id, 1, tagBuffer->getDepthConversionCoefficient());
    Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA(0 ,1);
    std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations;
    tagBool_t used; // Could have been made standard vector, only need receiver address. But if we move toestimator?

    // Create unix timestamp in milliseconds for the most recent measurement
    long int measurement_ms = (long int)receiverBuffer->rbegin()->unix_timestamp*1000 + receiverBuffer->rbegin()->millis;

    tempTagBuffer.tagBuffer[receiver] = new tagBuffer_t(1);
    tempTagBuffer.tagBuffer[receiver]->push_back(*(receiverBuffer->rbegin()));
    // Depth reading from the current tag
    if(tagBuffer->getDepthConversionCoefficient() < 0.01) {
      tempTagBuffer.tagBuffer[receiver]->begin()->trans_data = std::abs(std::round(receiver_depth/tagBuffer->getDepthConversionCoefficient()));
    }
    int receiverAdder=1;

    // Check the buffer of older tag detections from the second newest to the oldest.
    // Only combine if a multiple of the period is found within a given threashold/jitter.
    for(tagBuffer_t::const_reverse_iterator i=receiverBuffer->rbegin()+1; i != receiverBuffer->rend();i++) {

       // Time difference of arrival without correcting for period
      long int rawTDOA_ms = measurement_ms - (long int)i->unix_timestamp*1000 - i->millis;

      long int tempTDOA_ms;
      if(interval_mode == interval_mode_fixed_unknown) {
        tempTDOA_ms = rawTDOA_ms - std::round((double)rawTDOA_ms/1000)*1000;
      } else { // Fixed period corrigated time difference of arrival
        long int closestMultipleOfPeriod_ms = tag_period*1000*std::round(rawTDOA_ms/(tag_period*1000));
        tempTDOA_ms = rawTDOA_ms - closestMultipleOfPeriod_ms;
      }
      // Check if buffered detection satisfies conditions for use in estimator
      if(std::abs(tempTDOA_ms) < max_jitter*1000 && std::abs(tempTDOA_ms) > 0) {   
        // Add to temporary tagBuffer
        tempTagBuffer.tagBuffer[receiver+receiverAdder] = new tagBuffer_t(1);
        tempTagBuffer.tagBuffer[receiver+receiverAdder]->push_back(*i);

        RDOAcombinations.push_back(std::pair<uint32_t, uint32_t>(receiverBuffer->rbegin()->serial_no, receiver+receiverAdder));
        RDOA.conservativeResize(RDOA.rows()+1,1);
        RDOA(RDOA.rows()-1,0) = -1*c_speed*tempTDOA_ms/1000;
        // Depth reading from the current tag
        if(tagBuffer->getDepthConversionCoefficient() < 0.01) {
          tempTagBuffer.tagBuffer[receiver+receiverAdder]->begin()->trans_data = std::abs(std::round(receiver_depth/tagBuffer->getDepthConversionCoefficient()));
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

    //std::cout << "RDOA" << std::endl << RDOA << std::endl;
    //std::cout << "RDOAcombinations" << std::endl;
    //for(auto const &it : RDOAcombinations) {
    //  std::cout << it.first << " - " << it.second << std::endl;
    //}
    // Run filter update, and if sucessfull, set unprocessedData to false for used data receivers
    if(aslv.update(&tempTagBuffer, RDOA, RDOAcombinations)) {
      latestTimestamp = ((uint64_t)tagBuffer->tagBuffer.begin()->second->rbegin()->unix_timestamp)*1000 + tagBuffer->tagBuffer.begin()->second->rbegin()->millis;
      activateEstimator();
      return true;
    }
    return false;
  }
  void SingleReceiverASLV2::print(std::ostream& os) const {

    os << "x" << std::endl << aslv.xHat << std::endl;
  }
}