#include <boost/circular_buffer.hpp>
#include <vector>
namespace FishTagEstimators
{
    //! @brief Class that finds the transmission period of pseudo-randomly transmitting acoustic fish telemetry tags based on a known series of intervals.
    //! @author Nikolai Lauvås
    class PeriodFinder
    {
    public:
      //! Enumeration of the states possible for the expected next interval.
      enum class IntervalValidity : uint8_t
      {
        Invalid = 0,
        Valid = 1,
        LowestEstimate = 2
      };
      //! String containing comma separated transmission intervals
      //const std::string transmissionIntervals;
      const std::vector<uint16_t> transmissionIntervals;
      //! The minimum transmitting interval for the acoustic tranmsitter
      const uint16_t minInterval;
      //! The maximum transmitting interval for the acoustic tranmsitter
      const uint16_t maxInterval;
      //! The length of the pattern that is searched for
      const uint8_t patternSize;
      //! The maximum number of tries to consolidate unexpected intervals
      const uint16_t maxResolvingAttempts;

      PeriodFinder(const std::string &intervalsIn,
                    uint16_t minIntervalIn,
                    uint16_t maxIntervalIn,
                    uint8_t patternSizeIn = 5,
                    uint16_t maxResolvingAttemptsIn = 7) : 
            transmissionIntervals(stringToVector(intervalsIn)),
            minInterval(minIntervalIn),
            maxInterval(maxIntervalIn),
            patternSize(patternSizeIn),
            maxResolvingAttempts(maxResolvingAttemptsIn),
            intervalsBuffer(patternSizeIn),
            expectedIntervalStatus(IntervalValidity::Invalid),
            expectedInterval(0)
      {
        // Other initialization if needed
        previousIntervalBufferPosition = transmissionIntervals.cbegin();
      }

      PeriodFinder(const std::vector<uint16_t> &intervalsIn,
                    uint16_t minIntervalIn,
                    uint16_t maxIntervalIn,
                    uint8_t patternSizeIn = 5,
                    uint16_t maxResolvingAttemptsIn = 7) : 
            transmissionIntervals(intervalsIn),
            minInterval(minIntervalIn),
            maxInterval(maxIntervalIn),
            patternSize(patternSizeIn),
            maxResolvingAttempts(maxResolvingAttemptsIn),
            intervalsBuffer(patternSizeIn),
            expectedIntervalStatus(IntervalValidity::Invalid),
            expectedInterval(0)
      {
        // Other initialization if needed
        previousIntervalBufferPosition = transmissionIntervals.cbegin();
      }

      //! Update the current interval so that the expected interval can be updated
      //! @param[in] currentInterval A new transmission interval
      //! @return True if the next interval is definitively known, false if invalid or guessed.
      bool addInterval(uint16_t currentInterval);

      //! Returns the most recent expected interval. Returns the minimum interval if not known
      uint16_t getExpectedInterval() const
      {
        return expectedInterval;
      }

      //! Returns the current status of the expected interval
      IntervalValidity checkValidity() const
      {
        return expectedIntervalStatus;
      }

      //! Resets the variables associated with the expected interval
      void resetExpectedInterval();

    private:
      //! Buffer that stores most recent valid tag intervals used for matching
      boost::circular_buffer<uint16_t> intervalsBuffer;
      //! The position in the intervals string for the last known measured-interval match
      std::vector<uint16_t>::const_iterator previousIntervalBufferPosition;
      //! Tells if expectedInterval is expected to hold valid data
      IntervalValidity expectedIntervalStatus;
      //! The expected interval for the next tag reception
      uint16_t expectedInterval;

      //! 
      //! @param[in] currentInterval A new transmission interval
      //! @return True if the next interval is definitively known, false if invalid or guessed.
      bool handleExpectedInterval();

      //! Tries to consolidate out of bounds or unexpeted interval misses.
      //! Works by examining the sum of the next intervals to see if number of lost detections can be detected.
      //! @param[in] currentInterval A new transmission interval
      //! @return True if the next interval is definitively known, false if invalid or guessed.
      bool handleUnexpectedInterval(uint16_t currentInterval);

      uint16_t stringToUint16(const std::string& str) const;
      std::vector<uint16_t> stringToVector(const std::string& input);
      std::vector<std::vector<uint16_t>::const_iterator> findAllSequenceMatches();
      //! AI coded
      std::vector<std::vector<uint16_t>> findAllConsecutiveSequences(const std::vector<uint16_t>& numbers, uint16_t target_sum, uint16_t sequence_length);
    };
}