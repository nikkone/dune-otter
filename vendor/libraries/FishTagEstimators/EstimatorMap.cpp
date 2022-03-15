#include "EstimatorMap.hpp"
#include "SingleReceiverEKF.hpp"
#include "SingleReceiverUKF.hpp"
#include "SingleReceiverSRUKF.hpp"
#include "MultipleReceiverEKF.hpp"

namespace FishTagEstimators
{
  Estimator* EstimatorMap::addEstimator(uint32_t trans_id, estimatorTypeEnum_t type) {
    if(estimatorMap.find(trans_id) == estimatorMap.end()) {
      // New transmitterid found, create vector for estimators
      estimatorMap[trans_id] = new EstimatorVector_t();
    }
    Estimator* est;
    switch(type) {
        case estimatorType_SingleReceiverEKF:
          est = new SingleReceiverEKF();
          break;
        case estimatorType_SingleReceiverUKF:
          est = new SingleReceiverUKF();
          break;
        case estimatorType_SingleReceiverSRUKF:
          est = new SingleReceiverSRUKF();
          break;
        case estimatorType_MultipleReceiverEKF:
          est = new MultipleReceiverEKF();
          break;
        default:
          return nullptr;
    }
    estimatorMap[trans_id]->push_back(est);
    return est;
  }

  void EstimatorMap::updateAll(uint32_t trans_id, TagBuffer *tagBuffer) {
    for(EstimatorVector_t::iterator it = estimatorMap[trans_id]->begin();it != estimatorMap[trans_id]->end();it++) {
      (*it)->update(tagBuffer);
    }
  }

  void EstimatorMap::predictAll() {
    for (EstimatorMap_t::iterator it = estimatorMap.begin(); it != estimatorMap.end(); it++)
    {
      if (it->second != NULL)
      {
        for(EstimatorVector_t::iterator est = it->second->begin();est != it->second->end();est++) {
          (*est)->predict();
        }
      }
    }
  }

  void EstimatorMap::predictAll(uint32_t trans_id) {
    for(EstimatorVector_t::iterator it = estimatorMap[trans_id]->begin();it != estimatorMap[trans_id]->end();it++) {
      (*it)->predict();
    }
  }

  void EstimatorMap::setSoundSpeed(float c_speed_in) {
    for (EstimatorMap_t::iterator it = estimatorMap.begin(); it != estimatorMap.end(); it++)
    {
      if (it->second != NULL)
      {
        for(EstimatorVector_t::iterator est = it->second->begin();est != it->second->end();est++) {
          (*est)->setSoundSpeed(c_speed_in);
        }
      }
    }
  }
}