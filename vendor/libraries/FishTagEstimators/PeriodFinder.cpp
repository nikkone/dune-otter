#include "PeriodFinder.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <algorithm>
namespace FishTagEstimators
{
  bool PeriodFinder::addInterval(uint16_t currentInterval)
  {
    // Rule out same detections of the same transmission, different receiver
    if (currentInterval <= 1)
      return false;

    // Check if the interval is outside the specified range or if it's unexpected
    if (currentInterval > maxInterval || currentInterval < minInterval || ((expectedIntervalStatus == IntervalValidity::Valid) && (currentInterval != expectedInterval)))
    {
      return handleUnexpectedInterval(currentInterval);
    }
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
    
    return handleExpectedInterval();
  }

  bool PeriodFinder::handleUnexpectedInterval(uint16_t currentInterval)
  {
    std::vector<uint16_t>::const_iterator temporaryIntervalPosition;
    uint32_t sum = 0;
    temporaryIntervalPosition = previousIntervalBufferPosition;
    for (uint16_t i = 0; i < maxResolvingAttempts; ++i)
    {
      ++temporaryIntervalPosition;
      if(temporaryIntervalPosition == transmissionIntervals.cend()) {
        temporaryIntervalPosition = transmissionIntervals.cbegin();
      }
      uint16_t nextNumber = *temporaryIntervalPosition;
      sum += nextNumber;
      intervalsBuffer.push_back(nextNumber);
      std::cout << "Lost: " << nextNumber << ", Sum: " << sum << std::endl;
      if (sum >= currentInterval)
      {
        break;
      }
    }

    
    if (sum == currentInterval)
    {
      // Match found after lost detections was accounted for
      previousIntervalBufferPosition = temporaryIntervalPosition;
      expectedInterval = *(++temporaryIntervalPosition);
      if (expectedInterval == 0)
      {
        expectedIntervalStatus = IntervalValidity::Invalid;
        return false;
      }
      expectedIntervalStatus = IntervalValidity::Valid;
      return true;
    }
    int maxSequenceLength = 20;
    if((expectedIntervalStatus == IntervalValidity::Invalid) && (currentInterval > maxInterval) && (currentInterval < maxSequenceLength*maxInterval)) {
        
        std::vector<std::vector<uint16_t>> allSequences;
        for (int sequenceLength = 3; sequenceLength <= maxSequenceLength; ++sequenceLength) {
            std::vector<std::vector<uint16_t>> sequences = findAllConsecutiveSequences(transmissionIntervals, currentInterval, sequenceLength);

            if (!sequences.empty()) {
                allSequences.insert(allSequences.end(), sequences.begin(), sequences.end());
            }
        }
        if(!allSequences.empty()) {
          if (allSequences.size() == 1) {
            std::cout << "Unique solution!" << std::endl;
            resetExpectedInterval();
            for(auto it : *(allSequences.begin())) {
              intervalsBuffer.push_back(it);
            }
            return handleExpectedInterval();
          } else {
            // TODO: Choose minval, but set lowest estimate state
            std::cout << "Multiple solutions!" << std::endl;
          }
        }

    }

    // No match found
    expectedIntervalStatus = IntervalValidity::Invalid;
    intervalsBuffer.clear();
    return false;
  }

  bool PeriodFinder::handleExpectedInterval()
  {
    uint16_t nextInterval = 0;
    auto sequenceMatches = findAllSequenceMatches();
    
    if (!sequenceMatches.empty())
    {
      //std::cout << "nextInterval not empty" << std::endl;
      // Complete of partial Sucess: Found solution in unmodified string
      previousIntervalBufferPosition = sequenceMatches[0]+ intervalsBuffer.size() -1;
      size_t index = std::distance(transmissionIntervals.begin(), previousIntervalBufferPosition);
      //std::cout << "Size: " << transmissionIntervals.size() << ", Next: " << index << std::endl; 
      if(index >= transmissionIntervals.size()) {
        previousIntervalBufferPosition = transmissionIntervals.cbegin()+ index-transmissionIntervals.size();
        nextInterval = transmissionIntervals[index-transmissionIntervals.size()+1];
      } else if ((previousIntervalBufferPosition+1) == transmissionIntervals.cend()) {
        nextInterval = transmissionIntervals[0];
      } else {
        nextInterval = *(previousIntervalBufferPosition+1);
      }

    }
    else
    {
      std::cout << "nextInterval empty" << std::endl;
      intervalsBuffer.clear();
      resetExpectedInterval();
      return false;
    }

    if (sequenceMatches.size() > 1)
    {
      //std::cout << "Multiple options" << std::endl;
      // Partial sucess: provide the LowestEstimate (safe time to move for vehicles)
      // Find the minimum next value
      std::vector<uint16_t>::const_iterator minIt = transmissionIntervals.cbegin();
      auto minElement = std::numeric_limits<uint16_t>::max(); // Initialize with max value
      for (auto sequence = sequenceMatches.begin();sequence != sequenceMatches.end();sequence++) {
          // Increment the iterator if it's not at the end
          std::vector<uint16_t>::const_iterator innterIt;
          if (sequence != sequenceMatches.end()) {
            innterIt = ++(*sequence);
          } else {
            innterIt = transmissionIntervals.begin();
          }

          // Compare the value pointed to by the iterator with the current minimum
          if (sequence != sequenceMatches.end() && *innterIt < minElement) {
              minElement = *innterIt;
              minIt = innterIt;
          }
      }

      expectedInterval = minElement;
      expectedIntervalStatus = IntervalValidity::LowestEstimate;
      if(minIt+1 == transmissionIntervals.cend()) {
        minIt = transmissionIntervals.cbegin();
      }
      previousIntervalBufferPosition = minIt -1;
      size_t index = std::distance(transmissionIntervals.begin(), previousIntervalBufferPosition);
      //std::cout << "Size: " << transmissionIntervals.size() << ", Next: " << index << std::endl; 
      if(index >= transmissionIntervals.size()) {
        previousIntervalBufferPosition = transmissionIntervals.cbegin()+ index-transmissionIntervals.size();
        nextInterval = transmissionIntervals[index-transmissionIntervals.size()+1];
      }
      
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

  std::vector<uint16_t> PeriodFinder::stringToVector(const std::string& input) {
      std::vector<uint16_t> result;
      std::stringstream ss(input);
      std::string token;
      while (std::getline(ss, token, ',')) {
          try {
              uint16_t value = stringToUint16(token);
              result.push_back(value);
          } catch (const std::exception& e) {
              std::cerr << "Error: " << e.what() << std::endl;
          }
      }
      return result;
  }
  // Function to find all occurrences of the sequence
  std::vector<std::vector<uint16_t>::const_iterator> PeriodFinder::findAllSequenceMatches() {

      std::vector<std::vector<uint16_t>::const_iterator> matches;

      // Iterate over intervalsBuffer
      for (auto bufferIter = transmissionIntervals.begin(); bufferIter != transmissionIntervals.end(); ++bufferIter) {
          auto sequenceIter = intervalsBuffer.begin();

          // Check if the current element matches the first element of the sequence
          if (*bufferIter == *sequenceIter) {
              auto tempBufferIter = bufferIter;
              auto found = true;

              // Check if the subsequent elements match the rest of the sequence
              for (++sequenceIter, ++tempBufferIter; sequenceIter != intervalsBuffer.end(); ++sequenceIter, ++tempBufferIter) {
                  // If buffer wraps around, reset to the beginning
                  if (tempBufferIter == transmissionIntervals.end()) {
                      tempBufferIter = transmissionIntervals.begin();
                  }

                  if (*tempBufferIter != *sequenceIter) {
                      found = false;
                      break;
                  }
              }

              // If the sequence is found, store the iterators to the match
              if (found) {
                  matches.push_back(std::vector<uint16_t>::const_iterator(bufferIter));
              }
          }
      }

      return matches;
  }

  void PeriodFinder::resetExpectedInterval() {
    expectedIntervalStatus = IntervalValidity::Invalid;
    expectedInterval = minInterval;
  }
  
  uint16_t PeriodFinder::stringToUint16(const std::string& str) const {
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

  std::vector<std::vector<uint16_t>> PeriodFinder::findAllConsecutiveSequences(const std::vector<uint16_t> &numbers, uint16_t target_sum, uint16_t sequenceLength)
  {
    std::vector<std::vector<uint16_t>> result;
    uint16_t current_sum = 0;

    // Calculate the initial sum of the first 'sequenceLength' elements
    for (uint16_t i = 0; i < sequenceLength; ++i)
    {
      current_sum += numbers[i];
    }

    // Check if the initial sequence sums up to the target
    if (current_sum == target_sum)
    {
      std::vector<uint16_t> sequence;
      for (uint16_t i = 0; i < sequenceLength; ++i)
      {
        sequence.push_back(numbers[i]);
      }
      result.push_back(sequence);
    }

    // Slide the window to find the consecutive sequences
    for (size_t i = sequenceLength; i < numbers.size() + sequenceLength; ++i)
    {
      // Slide the window by removing the first element and adding the next element
      current_sum = current_sum - numbers[(i - sequenceLength) % numbers.size()] + numbers[i % numbers.size()];
      // Check if the current sequence sums up to the target
      if (current_sum == target_sum)
      {
        std::vector<uint16_t> sequence;
        for (uint16_t j = (i - sequenceLength + 1) % numbers.size(); j <= i % numbers.size(); ++j)
        {
          sequence.push_back(numbers[j]);
        }
        result.push_back(sequence);
      }
    }
    return result;
  }
}