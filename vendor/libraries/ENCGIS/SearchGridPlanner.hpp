#ifndef ENCGIS_SEARCHGRIDALGORTHMS_HPP_INCLUDED
#define ENCGIS_SEARCHGRIDALGORTHMS_HPP_INCLUDED

#define SEARCHGRID_USEOPP_OMPL 1

#if SEARCHGRID_USEOPP_OMPL
#include <OMPL/setup.hpp>
#endif

#include "SearchGrid.hpp"
namespace ENCGIS
{
    /// @brief 
    class SearchGridPlanner {
        public:
      typedef enum
      {
        P_LOCAL_GREEDY_COST = 0,
        P_LOCAL_GREEDY_COST_AZIMUTH = 1,
        P_LOCAL_GREEDY_COST_DISTANCE = 2,
        P_LOCAL_GREEDY_COST_AZIMUTH_DISTANCE = 3,
        P_GLOBAL_GREEDY_COST = 4,
        P_GLOBAL_GREEDY_COST_AZIMUTH = 5,
        P_GLOBAL_GREEDY_COST_DISTANCE = 6,
        P_GLOBAL_GREEDY_COST_AZIMUTH_DISTANCE = 7,
      } planner_t;

        SearchGridPlanner(SearchGrid * inGrid) : grid(inGrid) {
        
        }

#if SEARCHGRID_USEOPP_OMPL
        /// @brief Uses a greedy algorithm to create a path covering all grid cells.
        /// @param startCell The first cell to be visited
        /// @return A vector of describing the cell visitation order
        std::vector<std::pair<double, double>> calculateSearchPath(int startCell, og::SimpleSetup &setup);
#endif

        /// @brief Uses a greedy algorithm to create a path covering all grid cells.
        /// @param startCell The first cell to be visited
        /// @return A vector of describing the cell visitation order
        std::vector<int> calculateSearchPath(int startCell);

        /// @brief Uses a greedy algorithm to create a path covering all grid cells. Penalizes azimuth changes.
        /// @param startCell The first cell to be visited
        /// @param initialAzimuth Azimuth at startCell
        /// @param azimuthWeight Absolute azimuth change is multiplied with this weight.
        /// @return A vector of describing the cell visitation order
        std::vector<int> calculateSearchPathAzimuth(int startCell, double initialAzimuth, double azimuthWeight = 0.1);

        /// @brief Uses a greedy algorithm to create a path covering all grid cells. TODO: UNDER DEVELOPMENT
        /// @param startCell The first cell to be visited
        /// @return A vector of describing the cell visitation order
        std::vector<int> calculateSearchPathDistance(int startCell);

        /// @brief Get the neighbor with the lowest weight
        /// @param cell The cell id for which to check the neighbors
        /// @return the neighbor with the lowest weight.
        int getLocalOptimalNeighbour(int cell);

        /// @brief Get the neighbor with the lowest weight and lowest change of azimuth
        /// @param cell The cell id for which to check the neighbors
        /// @param azimuth [in] previous azimuth [out] next azimuth
        /// @param azimuthWeight Absolute azimuth change is multiplied with this weight.
        /// @return The neighbor with the lowest weight and azimuth change.
        int getLocalOptimalNeighbourAzimuth(int cell, double &azimuth, double azimuthWeight);

        /// @brief Get the neighbor with the lowest weight also considering 
        /// @param cell The cell id for which to check the neighbors
        /// @return the neighbor with the lowest weight.
        int getDistanceOptimalNextCell(int cell);


        /// @brief Iterates through the cells of a path, removing cells that do not cause a change in azimuth.
        /// This is results in a lossless simplification, potentioaly reducing the number of waypoints required to form a path.
        /// @param cells The cell sequence representing the path
        /// @param acceptedAzimuthDeviation The azimuth deviation considered low enough to remove a cell
        /// @return A cell representation of the path with redundant cell specifications removed
        std::vector<int> removeRedundantCells(const std::vector<int> &cells, double acceptedAzimuthDeviation = 0.00000001);

        /// @brief 
        /// @param cell 
        /// @param azimuth 
        /// @param azimuthWeight 
        /// @param distanceWeight 
        /// @return 
        int getGlobalOptimalCell(int cell, double azimuth, double azimuthWeight = 0.001, double distanceWeight = 0.0003);

#if SEARCHGRID_USEOPP_OMPL
        /// @brief 
        /// @param startCell 
        /// @param setup 
        /// @param initialAzimuth 
        /// @param azimuthWeight 
        /// @param distanceWeight 
        /// @return 
        std::vector<std::pair<double, double>> calculateSearchPathGlobal(int startCell, og::SimpleSetup &setup, double initialAzimuth = 0.0, double azimuthWeight = 0.1, double distanceWeight = 0.1);
#endif
        /// @brief 
        /// @param startCell 
        /// @param initialAzimuth 
        /// @param azimuthWeight 
        /// @param distanceWeight 
        /// @return 
        std::vector<int> calculateSearchPathGlobal(int startCell, double initialAzimuth, double azimuthWeight = 0.1, double distanceWeight = 0.1);

        /// @brief 
        /// @param startX 
        /// @param startY 
        /// @param endX 
        /// @param endY 
        void setIntersectingCellsAsVisited(double startX, double startY, double endX, double endY);
        
        private:
            SearchGrid *grid;
    };
}
#endif