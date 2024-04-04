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
      //! How many detections are available for the most recent timestamp
      unsigned detectionsMostRecentTs;
      //! The timestamp of the first detection of the latest transmission.
      uint64_t latestTimestamp_ms;
      //! The timestep between the two most recent transmissions (ms).
      long latestTimeStep_ms;
    public:
      //! The data structure where tags are buffered
      tagBufferMap_t tagBuffer;
      
      //! Tag ID
      unsigned tag_id;
      //! The coefficient used to convert the data field of a tag to depth in meters
      float depthConversion;
      //! Constructor
      //! @param [in] tag_id_in The transmitter ID this buffer holds detections for.
      //! @param [in] buffer_size_in How many detections should the circular buffer hold for each receiver
      TagBuffer(unsigned tag_id_in, unsigned buffer_size_in, float depthConversion_in) : buffer_size(buffer_size_in), detectionsMostRecentTs(0), tag_id(tag_id_in), depthConversion(depthConversion_in) {};

      //! Destructor. Deletes all dynamically allocated memory the class has created
      virtual ~TagBuffer() {
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

      /// @brief Set the coefficient used to convert the data field of a tag to depth in meters
      /// @note If set to zero, estimator should not use depth measurement.
      /// @param depthConversion_inn The coefficient
      void setDepthConversionCoefficient(float depthConversion_inn);


      /// @brief Get the coefficient used to convert the data field of a tag to depth in meters
      /// @return The coefficient
     float getDepthConversionCoefficient() const ;


      //! Finds the most frequent period in the last numUsedDetections detections, which is assumed to be the regular intervall.
      //! @param [in] receiver serial number
      //! @param [in] numUsedDetections
      //! @return calculated regular period given in seconds [s].

      double calculateRegularPeriod(unsigned receiver, unsigned numUsedDetections = 3, unsigned minCount = 2) const;

      //! TBD
      //! @param [in] receiver serial number
      //! @return calculated current irregular period given in seconds [s].
      double calculateIrregularPeriod(unsigned receiver) const;

      /// @brief 
      /// @param period 
      /// @param tagPeriodMin 
      /// @param tagPeriodMax 
      /// @return 
      int checkPeriod(unsigned period, unsigned tagPeriodMin, unsigned tagPeriodMax) const;


      /// @brief 
      /// @param currentTimestamp 
      /// @return True if there has been updates within acceptable timeframe, false if not.
      virtual bool checkTimeout(double currentTimestamp);

      /// @brief Get function for latestTimestamp_ms
      /// @return latestTimestamp_ms
      uint64_t getLatestTimestamp() {
        return latestTimestamp_ms;
      }
      /// @brief Get function for latestTimeStep_ms
      /// @return latestTimeStep_ms
      long getLatestTimeStep() {
        return latestTimeStep_ms;
      }

      //!
      size_t size() const;

      unsigned getNoOfMostRecentdetections() {
        return detectionsMostRecentTs;
      }
  };

  //! A map of tag buffers, where the index is used for transmitter ID and the second is a pointer to a tagBuffers_t
  typedef std::map<unsigned, FishTagEstimators::TagBuffer*> tagBuffers_t;
}
#endif // FishTag_TagBuffer