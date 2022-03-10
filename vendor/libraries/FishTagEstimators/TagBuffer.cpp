
#include "TagBuffer.hpp"
#include <iostream>
namespace FishTagEstimators
{    
  void TagBuffer::printNewBool(void) {
    for (tagBool_t::iterator it = unprocessedData.begin(); it != unprocessedData.end(); it++)
    {
      if(it->second) {
        std::cout << "Receiver " << it->first << " True" << std::endl;
      } else {
        std::cout << "Receiver " << it->first << " False" << std::endl;
      }
    }
  }
}