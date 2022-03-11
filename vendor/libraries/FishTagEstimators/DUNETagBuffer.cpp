
#include "DUNETagBuffer.hpp"
#include <DUNE/Coordinates/WGS84.hpp>
#include <DUNE/Math/Angles.hpp>
namespace FishTagEstimators
{    

  void DUNETagBuffer::setReferenceCoordinate(double reference[2]) {
    refCoord[0] = DUNE::Math::Angles::radians(reference[0]);
    refCoord[1] = DUNE::Math::Angles::radians(reference[1]);
    refCoord[2] = 0.0;
  }

  void DUNETagBuffer::fromNEDframe(const double input[3], double (&output)[3]) {
    output[0] = refCoord[0];
    output[1] = refCoord[1];
    output[2] = refCoord[2];
    DUNE::Coordinates::WGS84::displace(input[0], input[1], input[2], &(output[0]), &(output[1]), &(output[2]));
  }


  //! Turns the latitude and longtitude of the input to a NED representation with refCoord as origin.
  //! @param [in/out] input Tag detection to take lat/lon [rad] from and save NED to
  void DUNETagBuffer::toNEDframe(TBRFishTag &input)
  {
    DUNE::Coordinates::WGS84::displacement(refCoord[0], refCoord[1], refCoord[2], input.lat, input.lon, 0.0, &(input.N), &(input.E), &(input.D));
  }

  bool
  DUNETagBuffer::addTagDetection(const DUNE::IMC::TBRFishTag* msg)
  {
    if(msg->trans_id == tag_id) {
      
      if(tagBuffer.find(msg->serial_no) == tagBuffer.end()) {
        // New receiver found, create buffer

        tagBuffer[msg->serial_no] = new tagBuffer_t(buffer_size);
        //spew("Created buffer for receiver %u", msg->serial_no);
      }
      TBRFishTag tag = toEstimatorTag(*msg);
      toNEDframe(tag);
      tagBuffer[msg->serial_no]->push_back(tag);
      
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