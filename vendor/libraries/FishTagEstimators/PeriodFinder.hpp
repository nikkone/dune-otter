#include <boost/circular_buffer.hpp>
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
    const std::string transmissionIntervals;
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
                  uint16_t maxResolvingAttemptsIn = 7)
        : transmissionIntervals(intervalsIn),
          minInterval(minIntervalIn),
          maxInterval(maxIntervalIn),
          patternSize(patternSizeIn),
          maxResolvingAttempts(maxResolvingAttemptsIn),
          intervalsBuffer(patternSizeIn),
          previousIntervalBufferPosition(0),
          expectedIntervalStatus(IntervalValidity::Invalid),
          expectedInterval(0)
    {
      // Other initialization if needed
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
    size_t previousIntervalBufferPosition;
    //! Tells if expectedInterval is expected to hold valid data
    IntervalValidity expectedIntervalStatus;
    //! The expected interval for the next tag reception
    uint16_t expectedInterval;

    //! 
    //! @param[in] currentInterval A new transmission interval
    //! @return True if the next interval is definitively known, false if invalid or guessed.
    bool handleExpectedInterval(uint16_t currentInterval);

    //! Tries to consolidate out of bounds or unexpeted interval misses.
    //! Works by examining the sum of the next intervals to see if number of lost detections can be detected.
    //! @param[in] currentInterval A new transmission interval
    //! @return True if the next interval is definitively known, false if invalid or guessed.
    bool handleUnexpectedInterval(uint16_t currentInterval);

    //! Finds the next number from a given position in a string of comma separated values.
    //! @param[in] input A string containing comma separated values
    //! @param[in] position A position in input
    //! @return The next number in input after position separated by commas
    uint16_t findNextNumber(const std::string &input, size_t &position);

    //! Generates a string containing the comma-separated values of the buffer that can be searched for in the a priori transmissionIntervals string.
    //! @param[in] currentIntervals The buffer containint the most recent intervals
    //! @return A string containing the comma-separated values of the buffer
    std::string getCurrentIntervalString(const boost::circular_buffer<uint16_t> &currentIntervals) const;

    //! Takes a string, finds the comma closest to the center, and shifts the string so that the end of the string is now in the center.
    //! @param[in] input String to that is to be shifted
    //! @return Shifted string
    std::string shiftString(const std::string &input) const;

    uint8_t findIntervalsInString(const std::string& substring, const std::string& str, uint16_t& minFound, size_t& prev_pos);
    uint16_t stringToUint16(const std::string& str) const;
  };
}