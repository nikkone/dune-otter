
#include "DUNETagBuffer.hpp"
#include <DUNE/Coordinates/WGS84.hpp>
#include <DUNE/Math/Angles.hpp>
namespace FishTagEstimators
{    

  void DUNETagBuffer::setReferenceCoordinateRad(const double (&reference)[3]) {
    refCoord[0] = reference[0];
    refCoord[1] = reference[1];
    refCoord[2] = reference[2];
  }
  void DUNETagBuffer::setReferenceCoordinateDeg(const double (&reference)[3]) {
    refCoord[0] = DUNE::Math::Angles::radians(reference[0]);
    refCoord[1] = DUNE::Math::Angles::radians(reference[1]);
    refCoord[2] = reference[2];
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
      TBRFishTag tag = toEstimatorTag(msg);
      toNEDframe(tag);
      tagBuffer[msg->serial_no]->push_back(tag);
      if(msg->unix_timestamp - latestTimestamp > maxTimestampDifference) {
        // TODO: Check if more than one unused, then run update of filter
        //latestTimestamp = msg->getTimeStamp();
        latestTimestamp = ((uint64_t)msg->unix_timestamp)*1000 + msg->millis;
        //std::cout << std::endl << "set" << std::endl;
      }
      return true;
    }
    // Ignore other tags
    return false;
  }

  TBRFishTag DUNETagBuffer::toEstimatorTag(const DUNE::IMC::TBRFishTag *tagIn) {
    TBRFishTag tagOut;
    tagOut.serial_no = tagIn->serial_no;
    tagOut.unix_timestamp = tagIn->unix_timestamp;
    tagOut.millis = tagIn->millis;
    tagOut.trans_protocol = tagIn->trans_protocol;
    tagOut.trans_id = tagIn->trans_id;
    tagOut.trans_data = tagIn->trans_data;
    tagOut.snr = tagIn->snr;
    tagOut.trans_freq = tagIn->trans_freq;
    tagOut.recv_mem_addr = tagIn->recv_mem_addr;
    tagOut.lat = tagIn->lat;
    tagOut.lon = tagIn->lon;
    return tagOut;
  }

  bool DUNETagBuffer::checkTimeout(double currentTimestamp) {
    if(timestampTimeoutLimit > currentTimestamp - latestTimestamp) {
      return true;
    } else {
      return false;
    }
    
  }
  
  void clearDUNETagBuffers_t(DUNETagBuffers_t *tagBuffers) {
    for (DUNETagBuffers_t::iterator it = tagBuffers->begin(); it != tagBuffers->end(); it++)
    {
      delete it->second;
    }
    tagBuffers->clear();
  }

}