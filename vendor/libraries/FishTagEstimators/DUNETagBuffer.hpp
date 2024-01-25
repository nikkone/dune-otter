#ifndef FishTag_DUNETagBuffer
#define FishTag_DUNETagBuffer
#include "TagBuffer.hpp"
#include <DUNE/IMC.hpp>
namespace FishTagEstimators
{    

    class DUNETagBuffer : public TagBuffer
    {
    public:
      //! Constructor
      //! @param [in] tag_id_in
      //! @param [in] buffer_size_in
      DUNETagBuffer(unsigned tag_id_in, unsigned buffer_size_in, float depthConversion_in) : TagBuffer(tag_id_in, buffer_size_in, depthConversion_in) {
        latestTimestamp = 0;
        timestampTimeoutLimit = 4000;
      };

      //! Adds a tag detection to the buffer.
      //! @param [in] msg Message to be added in buffer
      //! @return True if tag added, false if not
      bool addTagDetection(const DUNE::IMC::TBRFishTag* msg);

      //! Converts DUNE::IMC::TBRFishTag to format used in estimators.
      //! @param [in] tagIn IMC fish tag message
      //! @return Fish tag structure used in estimators
      TBRFishTag toEstimatorTag(const DUNE::IMC::TBRFishTag *tagIn);

      //! Sets the position to use as origin in local NED frame.
      //! @param [in] reference Reference coordinate in WGS84 {lat[rad], lon [rad], elevation [m]} 
      void setReferenceCoordinateRad(const double (&reference)[3]);

      //! Sets the position to use as origin in local NED frame.
      //! @param [in] reference Reference coordinate in WGS84 {lat[deg], lon [deg], elevation [m]} 
      void setReferenceCoordinateDeg(const double (&reference)[3]);

      //! Takes a NED frame position and transforms it to a WGS84 lat/lon/elevation position.
      //! @param [in] input NED frame position to transform {North, East, Down} [meters] relative to the reference coordinate
      //! @param [out] output Input position converted to WGS84 coordinates {lat[rad], lon [rad], elevation [m]} 
      void fromNEDframe(const double input[3], double (&output)[3]);

      //! Turns the latitude and longtitude of the input to a NED representation with refCoord as origin.
      //! @param [in/out] input Tag detection to take lat/lon [rad] from 
      void toNEDframe(TBRFishTag &input);

      /// @brief Get function for latestTimestamp
      /// @return latestTimestamp
      uint64_t getLatestTimestamp() {
        return latestTimestamp;
      }

      void setTimestampTimeoutLimit(uint32_t timestampTimeoutLimit_inn) {
        timestampTimeoutLimit = timestampTimeoutLimit_inn;
      }

      uint32_t getTimestampTimeoutLimit() {
        return timestampTimeoutLimit;
      }

      bool checkTimeout(double currentTimestamp);

      void getReferenceCoordinateRad(double (&reference)[3]) {
        reference[0] = refCoord[0];
        reference[1] = refCoord[1];
        reference[2] = refCoord[2];
      }

    protected:
      //! Coordinate used as origin in local NED frame
      double refCoord[3];

      //! The maximum time to consider the a transmission to be the same as last one
      const uint32_t maxTimestampDifference = 1;

      //! The maximum time to consider the a transmission to be the same as last one
      uint32_t timestampTimeoutLimit;

      //! The timestamp of the first detection of the latest transmission.
      uint64_t latestTimestamp;

  };
  //! A map of tag buffers, where the index is used for transmitter ID and the second is a pointer to a DUNETagBuffers_t
  typedef std::map<unsigned, FishTagEstimators::DUNETagBuffer*> DUNETagBuffers_t;
  //!
  //! [inout]
  void clearDUNETagBuffers_t(DUNETagBuffers_t *tagBuffers);
}
#endif // FishTag_TagBuffer