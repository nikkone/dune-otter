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
      std::string name;
      static const unsigned c_states = 3;
      virtual void update(const tagBufferMap_t &tagBuffer, tagBool_t &unprocessedData);
      virtual void predict();

      virtual std::tuple<double, double, double> getEstimate();
      void setSoundSpeed(double soundSpeed);
      
      void setAllowedTimeShift(double AllowedTimeShift);
      void setTDOACovariance(double TDOACovariance);
      void setDepthCovariance(double depthCovariance);

      void setReferenceCoordinate(double reference[2]);

      virtual void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn);

      //! Takes a NED frame position and transforms it to a WGS84 lat/lon/elevation position
      //! @param [in] input NED frame position to transform {North, East, Down} [meters] relative to the reference coordinate
      //! @param [in] refCoord Reference coordinate in WGS84 {lat[rad], lon [rad], elevation [m]} 
      //! @param [out] output Input position converted to WGS84 coordinates {lat[rad], lon [rad], elevation [m]} 
      void fromNEDframe(const double input[3], double (&output)[3]);

      virtual bool isActive() const;

      virtual void print(std::ostream& os) const;

      friend std::ostream& operator<<(std::ostream& os, const Estimator& es) {
        es.print(os);
        return os;
      }
      bool setParameter(std::string parameterName, double value);
    protected:
      //! Check if the TDOA indicates a time shift larger than accepted
      //! @param [in] TDOA Time Difference of Arrival 
      //! @return Boolean representing accepted/not accepted
      bool timeShiftCorrect(const long int TDOA);

      //! Turns the latitude and longtitude of the input to a NED representation with refCoord as origin.
      //! @param [in] input Tag detection to take lat/lon [rad] from 
      //! @param [in] refCoord Reference coordinate in {lat[rad], lon [rad], elevation [m]} 
      //! @param [out] output NED frame representation of input in {North, East, Down} [meters] relative to the reference coordinate
      void toNEDframe(const TBRFishTag &input, std::tuple<double, double, double> &output);

      void registerParameter(std::string parameterName);

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