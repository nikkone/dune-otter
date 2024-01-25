#include "Estimator.hpp"
#include <iostream>
  //! Base class for source position estimator algorithms on IMC::TBRFishTag
  //! @author Nikolai Lauvås
namespace FishTagEstimators
{
  template <class T>
   bool Estimator<T>::update(TagBuffer *tagBuffer) {
    std::cout << "Base. Buffersize:" << tagBuffer->tagBuffer.size() << "Unprocessed size" << unprocessedData.size() <<  std::endl;
    return false;
  }
    template <class T>
   void Estimator<T>::predict() {return;}

  template <class T>
   std::tuple<T, T, T> Estimator<T>::getEstimate() {
    return {0.0,0.0,0.0};
  }
    typedef enum 
  { 
    param_trans_id,
    param_receiver_depth,
    param_tag_period,
    param_max_jitter,
    param_max_updates_per_new_measurement,
    param_max_correction_attempts
  } t_param;
    template <class T>
  void Estimator<T>::setSoundSpeed(T soundSpeed) {
    c_speed = soundSpeed;
  }
    template <class T>
  void Estimator<T>::setAllowedTimeShift(T AllowedTimeShift) {
    max_time_shift_ms = AllowedTimeShift;
  }

  template <class T>
  void Estimator<T>::setTDOACovariance(T TDOACovariance) {
    rr_cov = TDOACovariance;
  }

    template <class T>
  void Estimator<T>::setDepthCovariance(T depthCovariance) {
    rz_cov = depthCovariance;
  }

  template <class T>
  Eigen::Matrix<T, Estimator<T>::c_states, 1> Estimator<T>::getNED(const TBRFishTag &tagIn) {
    Eigen::Matrix<T, c_states, 1> ret;
    ret << tagIn.N, tagIn.E, tagIn.D;
    return ret;
  }

  template <class T>
   void Estimator<T>::initialize(const Eigen::Matrix<T, c_states, c_states> &A_inn,
                                    const Eigen::Matrix<T, c_states, c_states> &Q_inn,
                                    const Eigen::Matrix<T, c_states, c_states> &P0_inn,
                                    const Eigen::Matrix<T, c_states, 1> &x0_inn)
  {
    std::cout << "A_inn" << std::endl << A_inn << std::endl;
    std::cout << "Q_inn" << std::endl << Q_inn << std::endl;
    std::cout << "P0_inn" << std::endl << P0_inn << std::endl;
    std::cout << "x0_inn" << std::endl << x0_inn << std::endl;
  }

  template <class T>
  void Estimator<T>::setPositionEstimate(const Eigen::Matrix<T, c_states, 1> &x0_inn) {
    std::cout << "x0_inn" << std::endl << x0_inn << std::endl;
  }

  template <class T>
  bool Estimator<T>::activateEstimator() {
    std::cout << "Base" << std::endl;
    return false;
  }

  template <class T>
   bool Estimator<T>::isActive() const {
    return false;
  }

  template <class T>
   void Estimator<T>::print(std::ostream& os) const {
    os << "Base";
  }

  /*std::ostream& Estimator<T>::operator<<(std::ostream& os, const Estimator& es) {
    es.print(os);
    return os;
  }*/

  template <class T>
  bool Estimator<T>::setParameter(std::string parameterName, T value) {
    if(parameters.find(parameterName) != parameters.end()) {
      parseParameter(parameters[parameterName], value);
      return true;
    }
    return false;
  }
  //! Check if the TDOA indicates a time shift larger than accepted
  //! @param [in] TDOA Time Difference of Arrival 
  //! @return Boolean representing accepted/not accepted
  template <class T>
  bool Estimator<T>::timeShiftCorrect(const long int TDOA)
  {
    if((std::abs(TDOA) <= max_time_shift_ms))
      return true;
    else
      return false;
  }
  template <class T>
  void Estimator<T>::registerParameter(std::string parameterName) {
    parameters[parameterName] = parameters.size();
  }

  template <class T>
  void Estimator<T>::registerParameter(std::string parameterName, unsigned id) {
    parameters[parameterName] = id;
  }

  template <class T>
   void Estimator<T>::parseParameter(unsigned parameterID, T value) {
    std::cout << "Error: Parameter " << parameterID << "caught at base with value" << value << std::endl;
  }

  template <class T>
  void Estimator<T>::updateUnprocessedData(uint32_t serial_no) {
    unprocessedData[serial_no] = true;
  }

  template <class T>
  void Estimator<T>::printNewBool(void) {
    for (tagBool_t::iterator it = unprocessedData.begin(); it != unprocessedData.end(); it++)
    {
      if(it->second) {
        std::cout << "Receiver " << it->first << " True" << std::endl;
      } else {
        std::cout << "Receiver " << it->first << " False" << std::endl;
      }
    }
  }
  template <class T>
  bool Estimator<T>::checkTime(T transmissionFirstTime, T currentTime) const {
    return true;
  }

}