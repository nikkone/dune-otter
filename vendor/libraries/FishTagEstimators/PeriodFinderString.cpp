#include "PeriodFinderString.hpp"
#include <cmath>
#include <iostream>
namespace FishTagEstimators
{
  bool PeriodFinderString::addInterval(uint16_t currentInterval)
  {
    // Rule out same detections of the same transmission, different receiver
    if (currentInterval <= 1)
      return false;

    // Check if the interval is outside the specified range or if it's unexpected
    if (currentInterval > maxInterval || currentInterval < minInterval || ((expectedIntervalStatus == IntervalValidity::Valid) && (currentInterval != expectedInterval)))
    {
      return handleUnexpectedInterval(currentInterval);
    }
    return handleExpectedInterval(currentInterval);
  }

  bool PeriodFinderString::handleUnexpectedInterval(uint16_t currentInterval)
  {
    // TODO: Handle invalid where currentInterval > maxInterval: No, multiple ambigous solutions to find_consequtive
    uint32_t sum = 0;
    size_t temporaryIntervalPosition = previousIntervalBufferPosition;
    for (uint16_t i = 0; i < maxResolvingAttempts; ++i)
    {
      uint16_t nextNumber = findNextNumber(transmissionIntervals, temporaryIntervalPosition);
      if (nextNumber == 0)
      {
        // Error handling for findNextNumber
        break;
      }
      sum += nextNumber;
      intervalsBuffer.push_back(nextNumber);
       //std::cout << "Lost: " << nextNumber << ", Sum: " << sum << std::endl;
      if (sum >= currentInterval)
      {
        break;
      }
    }
    if (sum == currentInterval)
    {
      // Match found after lost detections was accounted for
      previousIntervalBufferPosition = temporaryIntervalPosition;
      expectedInterval = findNextNumber(transmissionIntervals, temporaryIntervalPosition);
      if (expectedInterval == 0)
      {
        expectedIntervalStatus = IntervalValidity::Invalid;
        return false;
      }
      expectedIntervalStatus = IntervalValidity::Valid;
      return true;
    }
    else
    {
      // No match found
      expectedIntervalStatus = IntervalValidity::Invalid;
      intervalsBuffer.clear();
      return false;
    }
  }

  bool PeriodFinderString::handleExpectedInterval(uint16_t currentInterval)
  {
    // Start over again if expectedInterval provided, but was not sucessful
    if (expectedIntervalStatus != IntervalValidity::Invalid)
    {
      if (currentInterval != expectedInterval)
      {
        expectedIntervalStatus = IntervalValidity::Invalid;
        resetExpectedInterval();
      }
    }

    intervalsBuffer.push_back(currentInterval);
    std::string currentIntervals = getCurrentIntervalString(intervalsBuffer);

    size_t prevPos = 0;
    
    uint16_t minFound = maxInterval + 1;
    uint16_t nextInterval = 0;
    uint8_t found = findIntervalsInString(currentIntervals, transmissionIntervals, minFound, prevPos);

    if (found != 0)
    {
      // Complete of partial Sucess: Found solution in unmodified string
      previousIntervalBufferPosition = prevPos;
      nextInterval = findNextNumber(transmissionIntervals, prevPos);
    }
    else
    {
      // Nothing found in unmodified string, try rotating it to check end conditions
      std::string shiftedTransmissionIntervals = shiftString(transmissionIntervals);
      prevPos = 0;
      found = findIntervalsInString(currentIntervals, shiftedTransmissionIntervals, minFound, prevPos);
      if (found == 0)
      {
        // Failed: currentIntervals could not be found in transmissionIntervals
        intervalsBuffer.clear();
        resetExpectedInterval();
        return false;
      }
      // Sucess: Found solution in shifted transmissionIntervals string
      size_t mid = transmissionIntervals.length() / 2;
      size_t closestCommaIndex = transmissionIntervals.find_last_of(',', mid);
      previousIntervalBufferPosition = (prevPos+shiftedTransmissionIntervals.substr(closestCommaIndex).length())%transmissionIntervals.length();
      //std::cout << "Found in reversed! Pos: " << previousIntervalBufferPosition << std::endl;
      //std::cout << transmissionIntervals.substr(previousIntervalBufferPosition) << std::endl;
      nextInterval = findNextNumber(shiftedTransmissionIntervals, prevPos);
    }

    if (found > 1)
    {
      // Partial sucess: provide the LowestEstimate (safe time to move for vehicles)
      expectedInterval = minFound;
      expectedIntervalStatus = IntervalValidity::LowestEstimate;
      return false;
    }
      
    if (nextInterval != 0)
    {
      expectedInterval = nextInterval;
      expectedIntervalStatus = IntervalValidity::Valid;
      return true;
    }
    expectedIntervalStatus = IntervalValidity::Invalid;
    return false;
  }

  std::string PeriodFinderString::getCurrentIntervalString(const boost::circular_buffer<uint16_t> &currentIntervals) const{
    if (currentIntervals.empty()) {
        return "";
    }

    // Calculate the total size required for the string to avoid reallocations
    size_t totalSize = currentIntervals.size() * 4 - 1; // Max size string (three digits+one comma)*elements minus last comma removal 

    std::string currentIntervalsString;
    currentIntervalsString.reserve(totalSize); // Pre-allocate memory for the string

    for(const auto inter : currentIntervals) {
        currentIntervalsString += std::to_string(inter) + ",";
    }
    currentIntervalsString.pop_back(); // To remove last comma
    return currentIntervalsString;
  }

/* ChatGPT
Write cpp code takes a std::string with comma separated numbers shifts it according to the closest comma to center so that what comes after this point is now at the beginning, and what comes before this point is now at the end
*/
    std::string PeriodFinderString::shiftString(const std::string& input) const {
        // Find the closest comma to the center
        size_t mid = input.length() / 2;
        size_t closestCommaIndex = input.find_last_of(',', mid);

        // If no comma is found before the midpoint, consider the midpoint itself
        if (closestCommaIndex == std::string::npos)
            closestCommaIndex = mid;

        // Create a new string with the portion after the closest comma followed by the portion before it
        std::string shiftedString = input.substr(closestCommaIndex + 1) + "," + 
                                    input.substr(0, closestCommaIndex);

        return shiftedString;
    }

/* ChatGPT
In cpp, I have a large std::string that contains comma separated numbers. I want to create a function that is able to get a position in the large string and find the next number. If the end is reached, then the next number from the start should be returned
*/
  //! Created with ChatGPT, but needed additional work for end handeling
  uint16_t PeriodFinderString::findNextNumber(const std::string& input, size_t& position) {
      size_t startPos = position;
      size_t length = input.length();
      
      // Skip any non-digit characters
      while (startPos < length && !isdigit(input[startPos])) {
          startPos++;
      }
      if (startPos >= length) {
        startPos = 0;
        while (startPos < length && !isdigit(input[startPos])) {
            startPos++;
        }
      }

      
      size_t endPos = startPos;
      // Find the end of the number
      while (endPos < length && isdigit(input[endPos])) {
          endPos++;
      }
      
      // If the end of the string is reached, wrap around
      if (endPos > length) {
          endPos = 0;
          while (endPos < position && isdigit(input[endPos])) {
              endPos++;
          }
      }
      
      // Extract the number
      uint16_t result =0;
      try{
        result = stringToUint16(input.substr(startPos, endPos - startPos));
      } catch(...) {
        return 0;
      }
      
      // Update position
      position = endPos;
      
      return result;
  }
  
  void PeriodFinderString::resetExpectedInterval() {
    expectedIntervalStatus = IntervalValidity::Invalid;
    expectedInterval = minInterval;
  }

  // Function to find occurrences of a substring within a string
  uint8_t PeriodFinderString::findIntervalsInString(const std::string& substring, const std::string& str, uint16_t& minFound, size_t& prevPos)
  { 
    uint8_t found = 0;
    size_t pos = 0;
    while ((pos = str.find(substring, pos)) != std::string::npos)
    {

      prevPos = (pos + substring.length()) % str.length();
      pos += substring.length();
      found++;
      size_t tempPos = prevPos;
      minFound = std::min(minFound, findNextNumber(str, tempPos));
    }
    return found;
  }

  uint16_t PeriodFinderString::stringToUint16(const std::string& str) const {
      try {
          unsigned long value = std::stoul(str);
          if (value > std::numeric_limits<uint16_t>::max()) {
              throw std::out_of_range("Value is out of range for uint16_t");
          }
          return static_cast<uint16_t>(value);
      } catch (const std::exception& e) {
          // Handle conversion errors
          // You might want to log the error or throw a different exception
          // depending on your application's requirements
          return 0; // Return a default value or handle the error differently
      }
  }

}