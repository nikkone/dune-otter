#ifndef FishTag_Estimator
#define FishTag_Estimator
#include <vector>
#include <Eigen/Core>

#include "TagBuffer.hpp"
  //! Base class for source position estimator algorithms on TBRFishTag
  //! @author Nikolai Lauvås
namespace FishTagEstimators
{
  template <class T, uint8_t STATES = 3>
  class Estimator
  {
    public:
      //! Destructor, virtual so it calls innheritted destructor of subclass
      virtual ~Estimator() {}

      //! Estimator name used for distinguishing it in logfiles etc.
      std::string name;

      //! The amount of states in the process model of the estimator
      static const unsigned c_states = STATES;
      
      //! Transmitter ID this tag estimates the position for.
      uint32_t trans_id;

      //! Measurement update (a posteriori) of estimator
      virtual bool update(TagBuffer *tagBuffer);

      //! Time update (a priori) of estimator
      virtual void predict();

      //! Common interface to get interface.
      //! @return Tuple with estimated position in a NED frame (relative to origin used in tagBuffer).
      virtual std::tuple<T, T, T> getEstimate() const;

      virtual void estimateToStream(std::ostream& os) const;
      virtual void headerToStream(std::ostream& os) const;
      //! The current sound speed used for TDOA calculations
      void setSoundSpeed(T soundSpeed);

      //! set maximum allowed time [ms] shift between receivers' messages
      //! @param [in] AllowedTimeShift
      void setAllowedTimeShift(T AllowedTimeShift);

      //! Covariance used for TDOA measurements/calculations
      //! @param [in] TDOACovariance Covariance to use
      virtual void setTDOACovariance(T TDOACovariance);

      //! Covariance used for depth measurements
      //! @param [in] depthCovariance Covariance to use
      virtual void setDepthCovariance(T depthCovariance);

      //! Prepares the NED data of a tag detection for use in Eigen
      //! @param [in] tagIn Tag with NED of interest
      Eigen::Matrix<T, STATES, 1> getNED(const TBRFishTag &tagIn) const;

      //! Initializer for the estimator
      //! @param [in] A_inn State transition matrix
      //! @param [in] Q_inn Measurement covariance matrix
      //! @param [in] P0_inn Initial covarinace matrix
      //! @param [in] x0_inn Initial estimator position [NED]
      virtual void initialize(const Eigen::Matrix<T, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<T, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<T, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<T, c_states, 1> &x0_inn);

      //! Set initial estimator position
      //! @param [in] x0_inn Initial estimator position [NED]
      virtual void setPositionEstimate(const Eigen::Matrix<T, c_states, 1> &x0_inn);

      virtual void setSNRmodel(double B_fit_inn, double k_fit_inn, double a_fit_inn);

      //! Common interface to check if a estimator is considered active
      virtual bool isActive() const;

      //! Debugging function to show which receiver has unprocessed data
      void printNewBool(void) const;

      /// @brief Tell the estimator new data is available to be processed at next update
      /// @param serial_no 
      void updateUnprocessedData(uint32_t serial_no);

      bool hasUnprocessedData() const {
        for(auto receiver : unprocessedData) {
          if(receiver.second) {
            return true;
          }
        }
        return false;
      }
      
      /// @brief Set function for the filter predict timestep
      /// @param ts Timestep in seconds
      virtual void setTimestep(T ts) {
        if(ts > 0.0) {
          timestep = ts;
        }
      }

      /// @brief Get function for the filter predict timestep
      /// @return Timestep in seconds
      T getTimestep() {
        return timestep;
      }


      /// @brief Checks we have waited long enough to allow all receivers to report tag detection.
      /// @param currentTime Time since UNIX Epoch (Midnight UTC of January 1, 1970).
      /// @return True if we have waited long enough
      virtual bool checkTime(T transmissionFirstTime, T currentTime) const;

      //! Virtual function that allows different behavior of << operator for each subclass
      //! @param [inout] os The outputstream to write to
      virtual void print(std::ostream& os) const;

      //! Overload of the << functionality to use for printing estimator setup/state
      //! @param [inout] es Referenece to this estimator object to get data from
      //! @param [inout] os The outputstream to write to
      //! @return the ostream so that  << operators can be chained
      friend std::ostream& operator<<(std::ostream& os, const Estimator& es) {
        es.print(os);
        return os;
      }

      //! General and common interface for setting parameters in subclasses
      //! @return True if parameter found
      bool setParameter(std::string parameterName, T value);

      /// @brief Get function for the Geometric dillution of position of previous estimate
      /// @return GDOP
      uint64_t getGDOP() const {
        return GDOP;
      }

      /// @brief Get function for the receiver positions used in the previous estimate (North-East pairs) in the NED frame (relative to origin used in tagBuffer).
      /// @return GDOP
      std::vector<std::pair<T,T>> getUsedPositions() const{
        return latestReceiverPositionsUsed;
      }


    protected:
      //! Internal command to activate estimator
      //! @return True id activation successfull.
      virtual bool activateEstimator();

      //! Check if the TDOA indicates a time shift larger than accepted
      //! @param [in] TDOA Time Difference of Arrival 
      //! @return Boolean representing accepted/not accepted
      bool timeShiftCorrect(const long int TDOA) const;

      //! General and common interface for registering parameters in subclasses
      //! @param [in] parameterName Desired parameter name
      void registerParameter(std::string parameterName);

      //! General and common interface for registering parameters in subclasses
      //! @param [in] parameterName Desired parameter name
      //! @param [in] id Identifier associated with parametername
      void registerParameter(std::string parameterName, unsigned id);

      //! General and common interface for parsing parameters, overloadable in subclasses
      //! @param [in] parameterID Parameter identifier
      //! @param [in] value Value to set in parameter
      virtual void parseParameter(unsigned parameterID, T value);

      //! Map storing the registered parameters, and their associated identifiers.
      std::map<std::string, unsigned> parameters;

      //! Sound speed in water used for TDOA calculations
      T c_speed;

      //! Maximum allowed time [ms] shift between receivers' messages
      T max_time_shift_ms;

      //! Time of Arrival Covariance
      T rr_cov;

      //! Depth measurement Covariance
      T rz_cov;
      
      //! A datastructure to keep track of which tag detections have been used.
      tagBool_t unprocessedData;

      //! Geometric dillution of position of previous estimate
      uint32_t GDOP;

      //! Timestep that the predict step is run at in seconds
      T timestep;

      //! Positions used in the estimate currently provided by getEstimate
      std::vector<std::pair<T,T>> latestReceiverPositionsUsed;

  };
  template class Estimator<double>;
}
#endif // FishTag_Estimator