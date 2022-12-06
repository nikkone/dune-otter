#include "EstimatorMap.hpp"
#include "SingleReceiverEKF.hpp"
#include "SingleReceiverUKF.hpp"
#include "SingleReceiverSRUKF.hpp"
#include "MultipleReceiverEKF.hpp"
#include "MultipleReceiversXKF.hpp"
namespace FishTagEstimators
{
  Estimator<double>* EstimatorMap::addEstimator(uint32_t trans_id, estimatorTypeEnum_t type) {
    if(estimatorMap.find(trans_id) == estimatorMap.end()) {
      // New transmitterid found, create vector for estimators
      estimatorMap[trans_id] = new EstimatorVector_t();
    }
    Estimator<double>* est;
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
        case estimatorType_MultipleReceiverXKF:
          est = new MultipleReceiverXKF<double>();
          break;
        default:
          return nullptr;
    }
    estimatorMap[trans_id]->push_back(est);
    return est;
  }

  void EstimatorMap::updateAll(uint32_t trans_id, TagBuffer *tagBuffer) {
    if(estimatorMap.find(trans_id) != estimatorMap.end()) {
      for(EstimatorVector_t::iterator it = estimatorMap[trans_id]->begin();it != estimatorMap[trans_id]->end();it++) {
        (*it)->update(tagBuffer);
      }
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
    if(estimatorMap.find(trans_id) != estimatorMap.end()) {
      for(EstimatorVector_t::iterator it = estimatorMap[trans_id]->begin();it != estimatorMap[trans_id]->end();it++) {
        (*it)->predict();
      }
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
  void EstimatorMap::setParameterAll(std::string parameterName, double value) {
    for (EstimatorMap_t::iterator it = estimatorMap.begin(); it != estimatorMap.end(); it++)
    {
      if (it->second != NULL)
      {
        for(EstimatorVector_t::iterator est = it->second->begin();est != it->second->end();est++) {
          (*est)->setParameter(parameterName, value);
        }
      }
    }
  }

void EstimatorMap::updateUnprocessedDataAll(uint32_t serial_no) {
    for (EstimatorMap_t::iterator it = estimatorMap.begin(); it != estimatorMap.end(); it++)
    {
      if (it->second != NULL)
      {
        for(EstimatorVector_t::iterator est = it->second->begin();est != it->second->end();est++) {
          (*est)->updateUnprocessedData(serial_no);
        }
      }
    }
  }
  void EstimatorMap::clear() {
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
}