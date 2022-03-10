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

    };
  typedef std::map<unsigned, FishTagEstimators::DUNETagBuffer*> DUNETagBuffers_t;
}
#endif // FishTag_TagBuffer