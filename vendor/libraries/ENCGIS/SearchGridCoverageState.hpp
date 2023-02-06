#ifndef ENCGIS_SEARCHGRIDCOVERAGESTATE_HPP_INCLUDED
#define ENCGIS_SEARCHGRIDCOVERAGESTATE_HPP_INCLUDED
#include "SearchGrid.hpp"
namespace ENCGIS
{
    /// @brief 
    class SearchGridCoverageState : public SearchGrid{
    public:
        SearchGridCoverageState(ENCGIS::DBconnection *db, std::string dbGridTableName) : SearchGrid(db, dbGridTableName){
        }
        ~SearchGridCoverageState() {
             
        }

        /// @brief Simple update for sensor with definite range law 
        /// @param X Southing
        /// @param Y Northing
        /// @param range Radius of the circle around (X,Y) to consider searched
        /// @param timeReduction 
        /// @return True if query sucess, false if not
        bool update(float X, float Y, float range, float timeReduction = 0.1, std::string metric = "weight");

        /// @brief Update grid search states based logarithmic detection model
        /// @param X Southing
        /// @param Y Northing
        /// @param rpm actuation level
        /// @param timeReduction From acoustic transmitter model. Should be timestep/maxintervall of tag
        /// @param range range to calculate values around (X,Y)
        /// @return True if query sucess, false if not
        bool updateLogarithmic(float X, float Y, unsigned rpm, float timeReduction = 0.1, double range = 500, double pCutoff = 0.05, std::string metric = "weight");

        /// @brief Subtract a fixed value for all grid cells
        /// @param fixedDecrease Value to subtract for each cell
        /// @return True if query sucess, false if not
        bool decreaseAll(float fixedDecrease, std::string metric = "weight");


        /// @brief 
        /// @param fixedDecrease 
        /// @param decreaseFactor 
        /// @return 
        bool decreaseAll(float fixedDecrease, float decreaseFactor, std::string metric = "weight");

        /// @brief combines effort metric and prior metric and stores result in detection probability layer
        /// @param priorMetric 
        /// @param effortLayerMetric 
        /// @param detProbLayer 
        /// @return True if query sucess, false if not
        bool updateDetectionProbability(std::string detProbLayer, std::string priorMetric = "weight", std::string effortLayerMetric = "effort");
    private:

        /// @brief Function giving the coefficient for the logaritmic model for the FishOtters according to actuation level
        /// @param rpm actuation level
        /// @param b0 Output for parameter zero in logarithmic model
        /// @param b1 Output for parameter one in logarithmic model
        /// @return True if coefficients have been updated, False for too high RPM values.
        bool logarithmicModelCoefficients(unsigned rpm, double &b0, double &b1);

    };
}
#endif