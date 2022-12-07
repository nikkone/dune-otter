#ifndef FishTag_EstimatorMap
#define FishTag_EstimatorMap
#include <boost/circular_buffer.hpp>
#include <map>
#include <vector>
#include "Estimator.hpp"
namespace FishTagEstimators
{
  //! Class that stores an EstimatorMap_t and contains often used functions for it.
  //! @author Nikolai Lauvås
  class EstimatorMap
    {
    protected:
      //! 
      //unsigned buffer_size;
    public:
      //! 
      typedef std::vector<FishTagEstimators::Estimator<double>*> EstimatorVector_t;
      //! Used to store pairs of transmitter ID and active estimators on the tag.
      typedef std::map<uint32_t, EstimatorVector_t*> EstimatorMap_t;
      //! 
      typedef enum { 
        estimatorType_SingleReceiverEKF,
        estimatorType_SingleReceiverUKF,
        estimatorType_SingleReceiverSRUKF,
        estimatorType_MultipleReceiverEKF,
        estimatorType_MultipleReceiverXKF
      } estimatorTypeEnum_t;
      //! 
      EstimatorMap_t estimatorMap;
      //! 
      EstimatorMap() {
      };
      //! 
      ~EstimatorMap() {
        for (EstimatorMap_t::iterator it = estimatorMap.begin(); it != estimatorMap.end(); it++)
        {
          if (it->second != NULL)
          {
            for (EstimatorVector_t::iterator est = it->second->begin(); est != it->second->end(); est++) {
              delete *est;
            }
            delete it->second;
            it->second = NULL;
          }
        }
        estimatorMap.clear();
      }
      //! 
      void clear();
      //! 
      Estimator<double>* addEstimator(uint32_t trans_id, estimatorTypeEnum_t type);
      //! 
      void updateAll(uint32_t trans_id, TagBuffer *tagBuffer);
      //! 
      void predictAll();
      //! 
      void predictAll(uint32_t trans_id);
      //! 
      void setSoundSpeed(float c_speed_in);
      //!
      void setParameterAll(std::string parameterName, double value);
      //! 
      void updateUnprocessedDataAll(uint32_t serial_no);
    };
}
#endif // FishTag_EstimatorMap