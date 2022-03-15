#ifndef FishTag_EstimatorMap
#define FishTag_EstimatorMap
#include <boost/circular_buffer.hpp>
#include <map>
#include <vector>
#include "Estimator.hpp"
namespace FishTagEstimators
{
  //! Class that stores an EstimatorMap_t and contains often used functions for it.
  class EstimatorMap
    {
    protected:
      unsigned buffer_size;
    public:
      typedef std::vector<FishTagEstimators::Estimator*> EstimatorVector_t;
      //! Used to store pairs of transmitter ID and active estimators on the tag.
      typedef std::map<uint32_t, EstimatorVector_t*> EstimatorMap_t;
      typedef enum { 
        estimatorType_SingleReceiverEKF,
        estimatorType_SingleReceiverUKF,
        estimatorType_SingleReceiverSRUKF,
        estimatorType_MultipleReceiverEKF
      } estimatorTypeEnum_t;
      EstimatorMap_t estimatorMap;
      EstimatorMap() {
      };

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

    Estimator* addEstimator(uint32_t trans_id, estimatorTypeEnum_t type);

    void updateAll(uint32_t trans_id, TagBuffer *tagBuffer);
    void predictAll();
    void predictAll(uint32_t trans_id);
    void setSoundSpeed(float c_speed_in);

    };
  typedef std::map<unsigned, FishTagEstimators::EstimatorMap*> EstimatorMaps_t;
}
#endif // FishTag_EstimatorMap