
#include "DUNETagBuffer.hpp"
namespace FishTagEstimators
{    

  bool
  DUNETagBuffer::addTagDetection(const DUNE::IMC::TBRFishTag* msg)
  {
    if(msg->trans_id == tag_id) {
      
      if(tagBuffer.find(msg->serial_no) == tagBuffer.end()) {
        // New receiver found, create buffer

        tagBuffer[msg->serial_no] = new tagBuffer_t(buffer_size);
        //spew("Created buffer for receiver %u", msg->serial_no);
      }
      tagBuffer[msg->serial_no]->push_back(toEstimatorTag(*msg));
      unprocessedData[msg->serial_no] = true;
      //printNewBool();
      //for(std::vector<Estimator*>::iterator it = estimators.begin();it != estimators.end();it++) {
      //  (*it)->update(tagBuffer, unprocessedData);
      //}
      //printNewBool();
      return true;
    }
    // Ignore other tags
    return false;
  }

  TBRFishTag DUNETagBuffer::toEstimatorTag(DUNE::IMC::TBRFishTag tagIn) {
    TBRFishTag tagOut;
    tagOut.serial_no = tagIn.serial_no;
    tagOut.unix_timestamp = tagIn.unix_timestamp;
    tagOut.millis = tagIn.millis;
    tagOut.trans_protocol = tagIn.trans_protocol;
    tagOut.trans_id = tagIn.trans_id;
    tagOut.trans_data = tagIn.trans_data;
    tagOut.snr = tagIn.snr;
    tagOut.trans_freq = tagIn.trans_freq;
    tagOut.recv_mem_addr = tagIn.recv_mem_addr;
    tagOut.lat = tagIn.lat;
    tagOut.lon = tagIn.lon;
    return tagOut;
  }

}