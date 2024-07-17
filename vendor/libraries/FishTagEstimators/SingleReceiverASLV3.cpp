#include "SingleReceiverASLV3.hpp"
namespace FishTagEstimators
{
  std::tuple<double, double, double> SingleReceiverASLV3::getEstimate() const {
    return {aslv.x(0),aslv.x(1),aslv.x(2)};
  }

  void SingleReceiverASLV3::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    SingleReceiverBase::initialize(A_inn, Q_inn, P0_inn, x0_inn);
    name = "SingleReceiverASLV3";
  }

  bool SingleReceiverASLV3::update(TagBuffer *tagBuffer) {
    SingleReceiverBase::update(tagBuffer);
    if(tag_period <= 0) {
      return false;
    }
    if(tagBuffer->tagBuffer.find(receiver) ==tagBuffer->tagBuffer.end()) {
      std::cout << "Did not find receiver: " << receiver << std::endl;
      return false;
    }
    
    const tagBuffer_t *receiverBuffer = tagBuffer->tagBuffer.find(receiver)->second;
        unsigned int m_max_correction_attempts;
    if(max_correction_attempts == 0)
      m_max_correction_attempts = receiverBuffer->max_size()-2;
    else {
      m_max_correction_attempts = max_correction_attempts;
    }
    //inf("Update %ld", c_buffer_size - receiverBuffer->size());
    // Create unix timestamp in milliseconds for the most recent measurement
    
    //tagBuffer_t::const_reverse_iterator it = tagBuffer.find(receiver)->second->rbegin();
    long int measurement_ms = (long int)receiverBuffer->rbegin()->unix_timestamp*1000 + receiverBuffer->rbegin()->millis;

    unsigned int updates = 0;
    unsigned int attempt = 0;
    // Check the buffer of older tag detections from the second newest to the oldest.
    // Only combine if a multiple of the period is found within a given threashold/jitter.
    //std::cout << "Steg0"<< std::endl;
    for(tagBuffer_t::const_reverse_iterator i=receiverBuffer->rbegin()+1; i != receiverBuffer->rend();i++) {
      attempt++;
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
        double rangeSNR = 50; //(DELETE)
        // Range difference of arrival calculation
        double rdoa = c_speed*tempTDOA_ms/1000;
        // Depth reading from the current tag
        double depth;
        if(tagBuffer->getDepthConversionCoefficient() > 0.01) {
          depth = i->trans_data*tagBuffer->getDepthConversionCoefficient();
        } else {
          depth = receiver_depth;
        }
        // Compile all mesurements for convenience
        Eigen::Matrix<double, 9, 1> allMeasurements;
        
        allMeasurements << i->N, i->E ,receiver_depth, receiverBuffer->rbegin()->N,receiverBuffer->rbegin()->E ,receiver_depth, rdoa, rangeSNR, depth;

          // Update the algebraic solver
          //if (aslv.addMeasurement(allMeasurements)) {
            if(!isActive()) {
              activateEstimator();
            }
          //}
        

          updates++;
          // Stop the loop after using the new measurement a given number of times.
          if(updates >= max_updates_per_new_measurement) {
            return true; 
          }

        // Stop after a number of predetermined attempts
        if(attempt >= m_max_correction_attempts) {
          //war("Stopped because maximum correction attempts reached. Attempts: %u, Updates %d", attempt, updates);
          return false;
        }
      }
    }
    return false;
  }
/*
  void SingleReceiverASLV3::predict() {
    //ekf.predict();
  }*/
  void SingleReceiverASLV3::print(std::ostream& os) const {

    os << "x" << std::endl << aslv.x << std::endl;
  }
}