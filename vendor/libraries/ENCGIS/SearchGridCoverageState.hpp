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
        /// @param range 
        /// @return True if query sucess, false if not
        bool update(float X, float Y, float range);

        /// @brief Update grid search states based logarithmic detection model
        /// @param X Southing
        /// @param Y Northing
        /// @param rpm actuation level
        /// @param range range to calculate values around (X,Y)
        /// @return True if query sucess, false if not
        bool updateLogarithmic(float X, float Y, unsigned rpm, double range = 500);

        /// @brief Subtract a fixed value for all grid cells
        /// @param fixedDecrease Value to subtract for each cell
        /// @return True if query sucess, false if not
        bool decreaseAll(float fixedDecrease);


        /// @brief 
        /// @param fixedDecrease 
        /// @param decreaseFactor 
        /// @return 
        bool decreaseAll(float fixedDecrease, float decreaseFactor);
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