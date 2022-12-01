#ifndef FishTag_TagBuffer
#define FishTag_TagBuffer
#include <boost/circular_buffer.hpp>
#include <map>
#include "TBRFishTag.hpp"
namespace FishTagEstimators
{    
  //! A map datastructure to keep track of which tag detections have been used. (Needs furter investigation and failproofing)
  typedef std::map<uint32_t, bool> tagBool_t;

  //! A circular buffer that stores the tag messages
  typedef boost::circular_buffer<TBRFishTag> tagBuffer_t;

  //! A map of tag buffers, where the index is used to store the receiver serial number,
  //! and the second is a pointer to a circular buffer with received transmitter messages.
  typedef std::map<uint32_t, tagBuffer_t*> tagBufferMap_t;

  //! Class to wrap around a tagBufferMap_t. Contains related routines that makes it easier to work with in.
  class TagBuffer
  {
    protected:
      //! How many detections should the circular buffers hold for each receiver
      unsigned buffer_size;
    public:
      //! A datastructure to keep track of which tag detections have been used. (Needs furter investigation and failproofing)
      tagBool_t unprocessedData;

      //! The data structure where tags are buffered
      tagBufferMap_t tagBuffer;
      
      //! Tag ID
      unsigned tag_id;

      //! Constructor
      //! @param [in] tag_id_in The transmitter ID this buffer holds detections for.
      //! @param [in] buffer_size_in How many detections should the circular buffer hold for each receiver
      TagBuffer(unsigned tag_id_in, unsigned buffer_size_in) : buffer_size(buffer_size_in), tag_id(tag_id_in) {};

      //! Destructor. Deletes all dynamically allocated memory the class has created
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

      //! Debugging function to show which receiver has unprocessed data
      void printNewBool(void);

      //! Finds the most frequent period in the last numUsedDetections detections, which is assumed to be the regular intervall.
      //! @param [in] receiver serial number
      //! @param [in] numUsedDetections
      //! @return calculated regular period given in seconds [s].

      double calculateRegularPeriod(unsigned receiver, unsigned numUsedDetections = 3, unsigned minCount = 2);

      //! TBD
      //! @param [in] receiver serial number
      //! @return calculated current irregular period given in seconds [s].
      double calculateIrregularPeriod(unsigned receiver);

      /// @brief 
      /// @param period 
      /// @param tagPeriodMin 
      /// @param tagPeriodMax 
      /// @return 
      int checkPeriod(unsigned period, unsigned tagPeriodMin, unsigned tagPeriodMax);

      //!
      size_t size();
  };

  //! A map of tag buffers, where the index is used for transmitter ID and the second is a pointer to a tagBuffers_t
  typedef std::map<unsigned, FishTagEstimators::TagBuffer*> tagBuffers_t;
}
#endif // FishTag_TagBuffer