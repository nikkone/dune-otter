#ifndef FishTag_TagBuffer
#define FishTag_TagBuffer
#include <boost/circular_buffer.hpp>
#include <map>
#include "TBRFishTag.hpp"
namespace FishTagEstimators
{    
    typedef std::map<uint32_t, bool> tagBool_t;
    typedef boost::circular_buffer<TBRFishTag> tagBuffer_t;
    typedef std::map<uint32_t, tagBuffer_t*> tagBufferMap_t;
    class TagBuffer
    {
    protected:
      unsigned buffer_size;
    public:
      tagBool_t unprocessedData;
      tagBufferMap_t tagBuffer;
      //! Tag ID
      unsigned tag_id;

      TagBuffer(unsigned tag_id_in, unsigned buffer_size_in) : buffer_size(buffer_size_in), tag_id(tag_id_in) {

      };

    ~TagBuffer() {
      for (tagBufferMap_t::iterator it = tagBuffer.begin(); it != tagBuffer.end(); it++)
      {
        if (it->second != NULL)
        {
          delete it->second;
          it->second = NULL;
        }
      }
      tagBuffer.clear();
    }

      void printNewBool(void);

    };
  typedef std::map<unsigned, FishTagEstimators::TagBuffer*> tagBuffers_t;
}
#endif // FishTag_TagBuffer