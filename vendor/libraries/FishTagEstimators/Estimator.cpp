#include "Estimator.hpp"
#include <iostream>
  //! Base class for source position estimator algorithms on IMC::TBRFishTag
  //! @author Nikolai Lauvås
namespace FishTagEstimators
{
  template <class T, uint8_t STATES>
   bool Estimator<T, STATES>::update(TagBuffer *tagBuffer) {
    std::cout << "Base. Buffersize:" << tagBuffer->tagBuffer.size() << "Unprocessed size" << unprocessedData.size() <<  std::endl;
    return false;
  }
    template <class T, uint8_t STATES>
   void Estimator<T, STATES>::predict() {return;}

  template <class T, uint8_t STATES>
   std::tuple<T, T, T> Estimator<T, STATES>::getEstimate() const{
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
    template <class T, uint8_t STATES>
  void Estimator<T, STATES>::setSoundSpeed(T soundSpeed) {
    c_speed = soundSpeed;
  }
    template <class T, uint8_t STATES>
  void Estimator<T, STATES>::setAllowedTimeShift(T AllowedTimeShift) {
    max_time_shift_ms = AllowedTimeShift;
  }

  template <class T, uint8_t STATES>
  void Estimator<T, STATES>::setTDOACovariance(T TDOACovariance) {
    rr_cov = TDOACovariance;
  }

    template <class T, uint8_t STATES>
  void Estimator<T, STATES>::setDepthCovariance(T depthCovariance) {
    rz_cov = depthCovariance;
  }

  template <class T, uint8_t STATES>
  Eigen::Matrix<T, STATES, 1> Estimator<T, STATES>::getNED(const TBRFishTag &tagIn) const{
    Eigen::Matrix<T, c_states, 1> ret;
    ret << tagIn.N, tagIn.E, tagIn.D;
    return ret;
  }

  template <class T, uint8_t STATES>
   void Estimator<T, STATES>::initialize(const Eigen::Matrix<T, c_states, c_states> &A_inn,
                                    const Eigen::Matrix<T, c_states, c_states> &Q_inn,
                                    const Eigen::Matrix<T, c_states, c_states> &P0_inn,
                                    const Eigen::Matrix<T, c_states, 1> &x0_inn)
  {
    std::cout << "A_inn" << std::endl << A_inn << std::endl;
    std::cout << "Q_inn" << std::endl << Q_inn << std::endl;
    std::cout << "P0_inn" << std::endl << P0_inn << std::endl;
    std::cout << "x0_inn" << std::endl << x0_inn << std::endl;
  }

  template <class T, uint8_t STATES>
  void Estimator<T, STATES>::setPositionEstimate(const Eigen::Matrix<T, c_states, 1> &x0_inn) {
    std::cout << "x0_inn" << std::endl << x0_inn << std::endl;
  }

  template <class T, uint8_t STATES>
  bool Estimator<T, STATES>::activateEstimator() {
    std::cout << "Base" << std::endl;
    return false;
  }

  template <class T, uint8_t STATES>
   bool Estimator<T, STATES>::isActive() const {
    return false;
  }

  template <class T, uint8_t STATES>
   void Estimator<T, STATES>::print(std::ostream& os) const {
    os << "Base";
  }

  /*std::ostream& Estimator<T, STATES>::operator<<(std::ostream& os, const Estimator& es) {
    es.print(os);
    return os;
  }*/

  template <class T, uint8_t STATES>
  bool Estimator<T, STATES>::setParameter(std::string parameterName, T value) {
    if(parameters.find(parameterName) != parameters.end()) {
      parseParameter(parameters[parameterName], value);
      return true;
    }
    return false;
  }
  //! Check if the TDOA indicates a time shift larger than accepted
  //! @param [in] TDOA Time Difference of Arrival 
  //! @return Boolean representing accepted/not accepted
  template <class T, uint8_t STATES>
  bool Estimator<T, STATES>::timeShiftCorrect(const long int TDOA) const
  {
    if((std::abs(TDOA) <= max_time_shift_ms))
      return true;
    else
      return false;
  }
  template <class T, uint8_t STATES>
  void Estimator<T, STATES>::registerParameter(std::string parameterName) {
    parameters[parameterName] = parameters.size();
  }

  template <class T, uint8_t STATES>
  void Estimator<T, STATES>::registerParameter(std::string parameterName, unsigned id) {
    parameters[parameterName] = id;
  }

  template <class T, uint8_t STATES>
   void Estimator<T, STATES>::parseParameter(unsigned parameterID, T value) {
    std::cout << "Error: Parameter " << parameterID << "caught at base with value" << value << std::endl;
  }

  template <class T, uint8_t STATES>
  void Estimator<T, STATES>::updateUnprocessedData(uint32_t serial_no) {
    unprocessedData[serial_no] = true;
  }

  template <class T, uint8_t STATES>
  void Estimator<T, STATES>::printNewBool(void) const{
    for (auto it : unprocessedData)
    {
      if(it.second) {
        std::cout << "Receiver " << it.first << " True" << std::endl;
      } else {
        std::cout << "Receiver " << it.first << " False" << std::endl;
      }
    }
  }
  template <class T, uint8_t STATES>
  bool Estimator<T, STATES>::checkTime(T transmissionFirstTime, T currentTime) const {
    return true;
  }

}