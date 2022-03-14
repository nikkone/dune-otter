
#include "DUNETagBuffer.hpp"
#include <DUNE/Coordinates/WGS84.hpp>
#include <DUNE/Math/Angles.hpp>
//#include <DUNE/DUNE.hpp>
namespace FishTagEstimators
{    

  void DUNETagBuffer::setReferenceCoordinateRad(double reference[2]) {
    refCoord[0] = reference[0];
    refCoord[1] = reference[1];
    refCoord[2] = 0.0;
  }
  void DUNETagBuffer::setReferenceCoordinateDeg(double reference[2]) {
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
/*
  //! Function for logging to an external file and dispatching the result as IMC
  //! @param [in] result The result to be logged. This is a NED value
  //! @param [in] in_logfilename Filename of file written to
  //! @param [in] logname Name used for the ID in the dispatched IMC::RemoteSensorInfo
  void DUNETagBuffer::logResult(Estimator* est, const std::string &in_logfilename, const std::string &logname) {
    double lati,longi;
    std::tuple<double, double, double>  estimate = est->getEstimate();
    double result[3] = {std::get<0>(estimate), std::get<1>(estimate), std::get<2>(estimate)};
    double latLon[3] = {0,0,0};
      tagBuffer->fromNEDframe(result, latLon);
      //est->fromNEDframe(result, latLon);
      lati=latLon[0], longi=latLon[1];

      // Send output to Neptus/DUNE log
      IMC::RemoteSensorInfo tagPosition;
      tagPosition.lat = lati;
      tagPosition.lon = longi;
      tagPosition.alt = -result[2];
      tagPosition.data = std::to_string(result[0]) + std::to_string(result[1]) + "," + std::to_string(result[2]);
      //tagPosition.data << result[0] << "," << result[1] << "," << result[2];
      tagPosition.id = logname + std::to_string(m_args.tag_id);
      dispatch(tagPosition);

      // External Logfile
      #if LOGFTOILE
      std::ofstream logOutStream;
      logOutStream.open(in_logfilename, std::fstream::app);
      if (logOutStream.good()) {
        logOutStream.precision(15);
          logOutStream << Clock::getSinceEpochMsec() << "," << result[0] << "," << result[1] << "," << result[2] << "," << DUNE::Math::Angles::degrees(lati) << "," << DUNE::Math::Angles::degrees(longi) << std::endl;
          //logOutStream << *est;
          logOutStream.close();
      }   
      #endif
      //spew("%s :New Kalman Estimate: (N,E,D,La,Lo)= %.15f,%.15f,%.15f,%.15f, %.15f", logname.c_str(), result[0], result[1], result[2],DUNE::Math::Angles::degrees(lati),DUNE::Math::Angles::degrees(longi));
    }
  }*/

}