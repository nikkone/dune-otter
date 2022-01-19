#ifndef FishTag_Estimator
#define FishTag_Estimator
#include <DUNE/DUNE.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <unistd.h>
#include <Eigen/Core>
#include <OpenFilterPack/KalmanFilterDynamic.hpp>
namespace SourceEstimators
{
  //! Base class for source position estimator algorithms on IMC::TBRFishTag
  //! @author Nikolai Lauvås
  namespace FishTag
  {
    class Estimator
    {
      public:
      static const unsigned c_states = 3;
      typedef std::map<uint32_t, bool> tagBool_t;
      typedef std::map<uint32_t, boost::circular_buffer<DUNE::IMC::TBRFishTag>*> tagBuffer_t;
        //Estimator() {};
        //~Estimator() {};
        virtual void update(const tagBuffer_t &tagBuffer, tagBool_t &unprocessedData) {
          std::cout << "Base. Buffersize:" << tagBuffer.size() << "Unprocessed size" << unprocessedData.size() <<  std::endl;
        };
        virtual void predict() {return;};

        virtual std::tuple<double, double, double> getEstimate() {
          return {0.0,0.0,0.0};
        }

        void setSoundSpeed(double soundSpeed) {
          c_speed = soundSpeed;
        };
        
        void setAllowedTimeShift(double AllowedTimeShift) {
          max_time_shift_ms = AllowedTimeShift;
        };
        void setTDOACovariance(double TDOACovariance) {
          rr_cov = TDOACovariance;
        };
        void setDepthCovariance(double depthCovariance) {
          rz_cov = depthCovariance;
        };

        void setReferenceCoordinate(double reference[2]) {
          refCoord[0] = DUNE::Math::Angles::radians(reference[0]);
          refCoord[1] = DUNE::Math::Angles::radians(reference[1]);
          refCoord[2] = 0.0;
        };

        virtual void initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                         const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                         const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                         const Eigen::Matrix<double, c_states, 1> &x0_inn)
        {
          std::cout << "A_inn" << std::endl << A_inn << std::endl;
          std::cout << "Q_inn" << std::endl << Q_inn << std::endl;
          std::cout << "P0_inn" << std::endl << P0_inn << std::endl;
          std::cout << "x0_inn" << std::endl << x0_inn << std::endl;
        };
        //! Takes a NED frame position and transforms it to a WGS84 lat/lon/elevation position
        //! @param [in] input NED frame position to transform {North, East, Down} [meters] relative to the reference coordinate
        //! @param [in] refCoord Reference coordinate in WGS84 {lat[rad], lon [rad], elevation [m]} 
        //! @param [out] output Input position converted to WGS84 coordinates {lat[rad], lon [rad], elevation [m]} 
        void fromNEDframe(const double input[3], double (&output)[3]) {
          output[0] = refCoord[0];
          output[1] = refCoord[1];
          output[2] = refCoord[2];
          WGS84::displace(input[0], input[1], input[2], &(output[0]), &(output[1]), &(output[2]));
        }

      virtual void print(std::ostream& os) const {
        os << "Base";
      };
      friend std::ostream& operator<<(std::ostream& os, const Estimator& es) {
        es.print(os);
        return os;
      }

      protected:
        //! Check if the TDOA indicates a time shift larger than accepted
        //! @param [in] TDOA Time Difference of Arrival 
        //! @return Boolean representing accepted/not accepted
        bool timeShiftCorrect(const long int TDOA)
        {
          if((std::abs(TDOA) <= max_time_shift_ms))
            return true;
          else
            return false;
        }

        //! Turns the latitude and longtitude of the input to a NED representation with refCoord as origin.
        //! @param [in] input Tag detection to take lat/lon [rad] from 
        //! @param [in] refCoord Reference coordinate in {lat[rad], lon [rad], elevation [m]} 
        //! @param [out] output NED frame representation of input in {North, East, Down} [meters] relative to the reference coordinate
        void toNEDframe(const DUNE::IMC::TBRFishTag &input, std::tuple<double, double, double> &output)
        {
          //inf("%f, %f",Math::Angles::degrees(input.lat), Math::Angles::degrees(input.lon));
          DUNE::Coordinates::WGS84::displacement(refCoord[0], refCoord[1], refCoord[2], input.lat, input.lon, 0.0, &(std::get<0>(output)), &(std::get<1>(output)), &(std::get<2>(output)));
          //std::get<2>(output) = 0.0;
        }

        double c_speed;
        double refCoord[3];
        double max_time_shift_ms;
        //! Time of Arrival Covariance
        double rr_cov;
        //! Depth measurement Covariance
        double rz_cov;
    };
    /*std::ostream& operator<<(std::ostream& os, const Estimator& es) {
      es.print(os);
      return os;
    }*/

  }
}
#endif // FishTag_Estimator