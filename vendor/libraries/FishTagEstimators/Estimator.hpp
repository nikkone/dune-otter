#ifndef FishTag_Estimator
#define FishTag_Estimator
#include <Eigen/Core>

#include "TagBuffer.hpp"
  //! Base class for source position estimator algorithms on IMC::TBRFishTag
  //! @author Nikolai Lauvås
namespace FishTagEstimators
{
  class Estimator
  {
    public:
      virtual ~Estimator() {}
      std::string name;
      static const unsigned c_states = 3;
      //virtual void update(const tagBufferMap_t &tagBuffer, tagBool_t &unprocessedData);
      virtual void update(TagBuffer *tagBuffer);

      virtual void predict();

      virtual std::tuple<double, double, double> getEstimate();
      void setSoundSpeed(double soundSpeed);
      
      void setAllowedTimeShift(double AllowedTimeShift);
      void setTDOACovariance(double TDOACovariance);
      void setDepthCovariance(double depthCovariance);

      Eigen::Matrix<double, c_states, 1> getNED(TBRFishTag &tagIn);

      virtual void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);

      virtual void setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn);

      virtual bool isActive() const;

      virtual void print(std::ostream& os) const;

      friend std::ostream& operator<<(std::ostream& os, const Estimator& es) {
        es.print(os);
        return os;
      }
      bool setParameter(std::string parameterName, double value);
      uint32_t trans_id;
    protected:
      virtual bool activateEstimator();
      //! Check if the TDOA indicates a time shift larger than accepted
      //! @param [in] TDOA Time Difference of Arrival 
      //! @return Boolean representing accepted/not accepted
      bool timeShiftCorrect(const long int TDOA);

      void registerParameter(std::string parameterName);
      void registerParameter(std::string parameterName, unsigned id);

      virtual void parseParameter(unsigned parameterID, double value);

      std::map<std::string, unsigned> parameters;
      double c_speed;
      double refCoord[3];
      double max_time_shift_ms;
      //! Time of Arrival Covariance
      double rr_cov;
      //! Depth measurement Covariance
      double rz_cov;

  };
}
#endif // FishTag_Estimator