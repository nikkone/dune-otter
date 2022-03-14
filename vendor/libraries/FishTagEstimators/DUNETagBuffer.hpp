#ifndef FishTag_DUNETagBuffer
#define FishTag_DUNETagBuffer
#include "TagBuffer.hpp"
#include <DUNE/IMC.hpp>
namespace FishTagEstimators
{    

    class DUNETagBuffer : public TagBuffer
    {
    public:

      DUNETagBuffer(unsigned tag_id_in, unsigned buffer_size_in) : TagBuffer(tag_id_in, buffer_size_in) {
      };

      bool
      addTagDetection(const DUNE::IMC::TBRFishTag* msg);

      TBRFishTag toEstimatorTag(DUNE::IMC::TBRFishTag tagIn);

      void setReferenceCoordinateRad(double reference[2]);
      void setReferenceCoordinateDeg(double reference[2]);



      //! Takes a NED frame position and transforms it to a WGS84 lat/lon/elevation position
      //! @param [in] input NED frame position to transform {North, East, Down} [meters] relative to the reference coordinate
      //! @param [in] refCoord Reference coordinate in WGS84 {lat[rad], lon [rad], elevation [m]} 
      //! @param [out] output Input position converted to WGS84 coordinates {lat[rad], lon [rad], elevation [m]} 
      void fromNEDframe(const double input[3], double (&output)[3]);

  //! Turns the latitude and longtitude of the input to a NED representation with refCoord as origin.
  //! @param [in] input Tag detection to take lat/lon [rad] from 
  //! @param [in] refCoord Reference coordinate in {lat[rad], lon [rad], elevation [m]} 
  //! @param [out] output NED frame representation of input in {North, East, Down} [meters] relative to the reference coordinate
  void toNEDframe(TBRFishTag &input);

    protected:
      double refCoord[3];
    };
  typedef std::map<unsigned, FishTagEstimators::DUNETagBuffer*> DUNETagBuffers_t;
}
#endif // FishTag_TagBuffer