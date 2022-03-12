
#include "TagBuffer.hpp"
#include <iostream>
#include <cmath>  
namespace FishTagEstimators
{    
  void TagBuffer::printNewBool(void) {
    for (tagBool_t::iterator it = unprocessedData.begin(); it != unprocessedData.end(); it++)
    {
      if(it->second) {
        std::cout << "Receiver " << it->first << " True" << std::endl;
      } else {
        std::cout << "Receiver " << it->first << " False" << std::endl;
      }
    }
  }

  double TagBuffer::calculateRegularPeriod(unsigned receiver, unsigned numUsedDetections) {
    tagBufferMap_t::iterator receiverBuffer = tagBuffer.find(receiver);
    if(receiverBuffer != tagBuffer.end()) {
      unsigned minCount = 2;
      unsigned usedDetections = 0;
      static std::map<double,unsigned> periodCount;//period as index, count as value. Overflow not handled, will only occur at extremely many tag detections before restarting (10^9 if 4 added each run)
      for(tagBuffer_t::const_reverse_iterator i=receiverBuffer->second->rbegin(); i != receiverBuffer->second->rend()-1;i++) {
        double measurement_millis = i->unix_timestamp + (double)i->millis/1000;
        // Time difference of arrival without correcting for period
        double td = measurement_millis - (i+1)->unix_timestamp - (double)(i+1)->millis/1000;
        double temp_period = std::round(td);

        // If not already in map, create item. If in map, add one.
        if(!(periodCount.emplace(temp_period,1)).second) {
          periodCount[temp_period]++;
        }
        // Check if buffered detection satisfies conditions for use in estimator
        if(usedDetections >=numUsedDetections) {
          break;
        }
      }
      // Find the most frequent period
      std::map<double,unsigned>::iterator best = std::max_element(periodCount.begin(),
        periodCount.end(),
        [] (const std::pair<double,unsigned>& a, const std::pair<double,unsigned>& b)->bool{ return a.second < b.second; }
      );
        //tag_period = best->first;
      std::cout << "Best - "<<best->first << " , " << best->second << "\n";
      if(best->second >= minCount) {
        return best->first;
      }
    }
    return -1;
  }

  double TagBuffer::calculateIrregularPeriod(unsigned receiver, unsigned numUsedDetections) { // Currently not functional
    tagBufferMap_t::iterator receiverBuffer = tagBuffer.find(receiver);
    if(receiverBuffer != tagBuffer.end()) {
      unsigned usedDetections = 0;
      for(tagBuffer_t::const_reverse_iterator i=receiverBuffer->second->rbegin(); i != receiverBuffer->second->rend()-1;i++) {
        double measurement_millis = i->unix_timestamp + (double)i->millis/1000;
        // Time difference of arrival without correcting for period
        double td = measurement_millis - (i+1)->unix_timestamp - (double)(i+1)->millis/1000;
        double temp_period = std::round(td);


        // Check if buffered detection satisfies conditions for use in estimator
        if(usedDetections >=numUsedDetections) {
          break;
        }
      }
    }
    return -1;
  }
}