#include "Estimator.hpp"
#include <iostream>
  //! Base class for source position estimator algorithms on IMC::TBRFishTag
  //! @author Nikolai Lauvås
namespace FishTagEstimators
{
   void Estimator::update(TagBuffer *tagBuffer) {
    std::cout << "Base. Buffersize:" << tagBuffer->tagBuffer.size() << "Unprocessed size" << tagBuffer->unprocessedData.size() <<  std::endl;
  }
   void Estimator::predict() {return;}

   std::tuple<double, double, double> Estimator::getEstimate() {
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

  void Estimator::setSoundSpeed(double soundSpeed) {
    c_speed = soundSpeed;
  }
  
  void Estimator::setAllowedTimeShift(double AllowedTimeShift) {
    max_time_shift_ms = AllowedTimeShift;
  }
  void Estimator::setTDOACovariance(double TDOACovariance) {
    rr_cov = TDOACovariance;
  }
  void Estimator::setDepthCovariance(double depthCovariance) {
    rz_cov = depthCovariance;
  }


  Eigen::Matrix<double, Estimator::c_states, 1> Estimator::getNED(const TBRFishTag &tagIn) {
    Eigen::Matrix<double, c_states, 1> ret;
    ret << tagIn.N, tagIn.E, tagIn.D;
    return ret;
  }

   void Estimator::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                    const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                    const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                    const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    std::cout << "A_inn" << std::endl << A_inn << std::endl;
    std::cout << "Q_inn" << std::endl << Q_inn << std::endl;
    std::cout << "P0_inn" << std::endl << P0_inn << std::endl;
    std::cout << "x0_inn" << std::endl << x0_inn << std::endl;
    //registerParameter("trans_id");
  }

  void Estimator::setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn) {
    std::cout << "x0_inn" << std::endl << x0_inn << std::endl;
  }
  bool Estimator::activateEstimator() {
    std::cout << "Base" << std::endl;
    return false;
  }

   bool Estimator::isActive() const {
    return false;
  }
   void Estimator::print(std::ostream& os) const {
    os << "Base";
  }

  /*std::ostream& Estimator::operator<<(std::ostream& os, const Estimator& es) {
    es.print(os);
    return os;
  }*/

  bool Estimator::setParameter(std::string parameterName, double value) {
    if(parameters.find(parameterName) != parameters.end()) {
      parseParameter(parameters[parameterName], value);
      return true;
    }
    return false;
  }
  //! Check if the TDOA indicates a time shift larger than accepted
  //! @param [in] TDOA Time Difference of Arrival 
  //! @return Boolean representing accepted/not accepted
  bool Estimator::timeShiftCorrect(const long int TDOA)
  {
    if((std::abs(TDOA) <= max_time_shift_ms))
      return true;
    else
      return false;
  }

  void Estimator::registerParameter(std::string parameterName) {
    parameters[parameterName] = parameters.size();
  }
  void Estimator::registerParameter(std::string parameterName, unsigned id) {
    parameters[parameterName] = id;
  }
   void Estimator::parseParameter(unsigned parameterID, double value) {
    std::cout << "Error: Parameter " << parameterID << "caught at base with value" << value << std::endl;
  }
}