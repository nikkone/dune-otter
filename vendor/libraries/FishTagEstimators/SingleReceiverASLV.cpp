#include "SingleReceiverASLV.hpp"
namespace FishTagEstimators
{
  std::tuple<double, double, double> SingleReceiverASLV::getEstimate() {
    return {aslv.x(0),aslv.x(1),aslv.x(2)};
  }

  void SingleReceiverASLV::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    SingleReceiverBase::initialize(A_inn, Q_inn, P0_inn, x0_inn);
    name = "SingleReceiverASLV";
  }

  bool SingleReceiverASLV::update(TagBuffer *tagBuffer) {
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
    double measurement_millis = receiverBuffer->rbegin()->unix_timestamp + (double)receiverBuffer->rbegin()->millis/1000;

    unsigned int updates = 0;
    unsigned int attempt = 0;
    // Check the buffer of older tag detections from the second newest to the oldest.
    // Only combine if a multiple of the period is found within a given threashold/jitter.
    //std::cout << "Steg0"<< std::endl;
    for(tagBuffer_t::const_reverse_iterator i=receiverBuffer->rbegin()+1; i != receiverBuffer->rend();i++) {
      attempt++;
      //inf("%d - %d", receiverBuffer->rbegin()->unix_timestamp, i->unix_timestamp);

      // Time difference of arrival without correcting for period
      double td = measurement_millis - i->unix_timestamp - (double)i->millis/1000;

      // Calculate closest multiple of period between the new measurement and the buffered detection
      double closestMultipleOfPeriod = tag_period*std::round(td/tag_period);

      // Period corrigated time difference of arrival
      double tdoa = td - closestMultipleOfPeriod;

      //inf("delta %f %f", td, closestMultipleOfPeriod);

      // Check if buffered detection satisfies conditions for use in estimator
      if(abs(tdoa) < max_jitter || td > 60.0) {
        //std::cout << "Steg1"<< std::endl;
        // Linear fit of SNR to range
        //double P[2] = {m_args.ranging_snr_fit[0], m_args.ranging_snr_fit[1]};
        //double rangeSNR = (receiverBuffer->rbegin()->snr - P[1])/P[0];
        double rangeSNR = 50; //(DELETE)
        // Range difference of arrival calculation
        double rdoa = c_speed*tdoa;

        // Depth reading from the current tag
        double depth;
        if(depthConversion > 0.01) {
          depth = i->trans_data*depthConversion;
        } else {
          depth = receiver_depth;
        }
        // Compile all mesurements for convenience
        Eigen::Matrix<double, 9, 1> allMeasurements;
        
        allMeasurements << i->N, i->E ,receiver_depth, receiverBuffer->rbegin()->N,receiverBuffer->rbegin()->E ,receiver_depth, rdoa, rangeSNR, depth;

          // Update the algebraic solver
          if (aslv.addMeasurement(allMeasurements)) {
            if(!isActive()) {
              activateEstimator();
            }
          }
        

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
  void SingleReceiverASLV::predict() {
    //ekf.predict();
  }*/
  void SingleReceiverASLV::print(std::ostream& os) const {

    os << "x" << std::endl << aslv.x << std::endl;
  }
}