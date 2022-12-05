
#include "TagBuffer.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>  
namespace FishTagEstimators
{    
  double TagBuffer::calculateRegularPeriod(unsigned receiver, unsigned numUsedDetections, unsigned minCount) {
    tagBufferMap_t::iterator receiverBuffer = tagBuffer.find(receiver);
    if(receiverBuffer != tagBuffer.end()) {
      unsigned usedDetections = 0;
      std::map<double,unsigned> periodCount;//period as index, count as value. Overflow not handled, will only occur at extremely many tag detections before restarting (10^9 if 4 added each run)
      for(tagBuffer_t::const_reverse_iterator i=receiverBuffer->second->rbegin(); i != receiverBuffer->second->rend()-1;i++) {
        double measurement_millis = i->unix_timestamp + (double)i->millis/1000;
        // Time difference of arrival without correcting for period
        double td = measurement_millis - (i+1)->unix_timestamp - (double)(i+1)->millis/1000;
        double temp_period = std::round(td);

        // If not already in map, create item. If in map, add one.
        if(!(periodCount.emplace(temp_period,1)).second) {
          periodCount[temp_period]++;
        }
        usedDetections++;
        if(usedDetections >=numUsedDetections) {
          break;
        }
      }
      std::cout << "periodCount" << periodCount.size() << std::endl;
      if(periodCount.size() <1) {
        return -1;
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

  size_t TagBuffer::size() {
    return tagBuffer.size();
  }
  int TagBuffer::checkPeriod(unsigned period, unsigned tagPeriodMin, unsigned tagPeriodMax) {
    if(period >= tagPeriodMin && period <= tagPeriodMax) {
      return period;
    }
    return -1;
  }

  double TagBuffer::calculateIrregularPeriod(unsigned receiver) {
    tagBufferMap_t::iterator receiverBuffer = tagBuffer.find(receiver);
    if(receiverBuffer != tagBuffer.end()) {
      if(receiverBuffer->second->size() >1) {
        double measurement_millis = receiverBuffer->second->rbegin()->unix_timestamp + (double)receiverBuffer->second->rbegin()->millis/1000;
        // Current timestamp - previous timestamp
        double period = std::round(measurement_millis - ((receiverBuffer->second->rbegin()+1)->unix_timestamp + (double)(receiverBuffer->second->rbegin()+1)->millis/1000));
        return period;
      }
    }
    return -1;
  }
}