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

      typedef enum
      {
        H_DISTRIBUTION = 0,
        H_DISTRIBUTION_AZIMUTH = 1,
        H_DISTRIBUTION_DISTANCE = 2,
        H_DISTRIBUTION_AZIMUTH_DISTANCE = 3,
      } heuristic_t;

        SearchGridPlanner(SearchGrid * inGrid) : grid(inGrid) {
        
        }

#if SEARCHGRID_USEOPP_OMPL
        std::vector<std::pair<double, double>> runPlanner(bool local, heuristic_t heuristic, og::SimpleSetup &setup) {
            if(local) {
                switch (heuristic)
                {
                case H_DISTRIBUTION:
                    return calculateSearchPath(setup);
                    break;
                case H_DISTRIBUTION_AZIMUTH:
                    break;
                case H_DISTRIBUTION_DISTANCE:
                    break;
                case H_DISTRIBUTION_AZIMUTH_DISTANCE:
                    return calculateSearchPathGlobal(setup);
                    break;                 
                default:
                    break;
                    return std::vector<std::pair<double, double>>();
                }
            }
        }
#else
        std::vector<int> runPlanner(bool local, heuristic_t heuristic) {
            if(local) {
                switch (heuristic)
                {
                case H_DISTRIBUTION:
                    return calculateSearchPath();
                    break;
                case H_DISTRIBUTION_AZIMUTH:
                    return calculateSearchPathAzimuth();
                    break;
                case H_DISTRIBUTION_DISTANCE:
                    return calculateSearchPathDistance();
                    break;
                case H_DISTRIBUTION_AZIMUTH_DISTANCE:
                    // Not implemented               
                default:
                    break;
                    return std::vector<int>();
                }
            } else { // Global
                switch (heuristic)
                {
                case H_DISTRIBUTION:
                    return std::vector<int>();
                    break;
                case H_DISTRIBUTION_AZIMUTH:
                    return std::vector<int>();
                    break;
                case H_DISTRIBUTION_DISTANCE:
                    return std::vector<int>();
                    break;
                case H_DISTRIBUTION_AZIMUTH_DISTANCE:
                     return calculateSearchPathGlobal();             
                default:
                    break;
                    return std::vector<int>();
                }
            } // 
        }
#endif
#if SEARCHGRID_USEOPP_OMPL
        /// @brief Uses a greedy algorithm to create a path covering all grid cells.
        /// @param startCell The first cell to be visited
        /// @return A vector of describing the cell visitation order
        std::vector<std::pair<double, double>> calculateSearchPath(og::SimpleSetup &setup);
#endif

        /// @brief Uses a greedy algorithm to create a path covering all grid cells.
        /// @param startCell The first cell to be visited
        /// @return A vector of describing the cell visitation order
        std::vector<int> calculateSearchPath();

        /// @brief Uses a greedy algorithm to create a path covering all grid cells. Penalizes azimuth changes.
        /// @param startCell The first cell to be visited
        /// @param initialAzimuth Azimuth at startCell
        /// @param azimuthWeight Absolute azimuth change is multiplied with this weight.
        /// @return A vector of describing the cell visitation order
        std::vector<int> calculateSearchPathAzimuth();

        /// @brief Uses a greedy algorithm to create a path covering all grid cells. TODO: UNDER DEVELOPMENT
        /// @param startCell The first cell to be visited
        /// @return A vector of describing the cell visitation order
        std::vector<int> calculateSearchPathDistance();

        /// @brief Get the neighbor with the lowest weight
        /// @param cell The cell id for which to check the neighbors
        /// @return the neighbor with the lowest weight.
        int getLocalOptimalNeighbour(int cell);

        /// @brief Get the neighbor with the lowest weight and lowest change of azimuth
        /// @param cell The cell id for which to check the neighbors
        /// @param azimuth [in] previous azimuth [out] next azimuth
        /// @param azimuthWeight Absolute azimuth change is multiplied with this weight.
        /// @return The neighbor with the lowest weight and azimuth change.
        int getLocalOptimalNeighbourAzimuth(int cell, double &azimuth);

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
        int getGlobalOptimalCell(int cell, double azimuth);

#if SEARCHGRID_USEOPP_OMPL
        /// @brief 
        /// @param startCell 
        /// @param setup 
        /// @param initialAzimuth 
        /// @param azimuthWeight 
        /// @param distanceWeight 
        /// @return 
        std::vector<std::pair<double, double>> calculateSearchPathGlobal(og::SimpleSetup &setup);
#endif
        /// @brief 
        /// @param startCell 
        /// @param initialAzimuth 
        /// @param azimuthWeight 
        /// @param distanceWeight 
        /// @return 
        std::vector<int> calculateSearchPathGlobal();

        /// @brief 
        /// @param startX 
        /// @param startY 
        /// @param endX 
        /// @param endY 
        void setIntersectingCellsAsVisited(double startX, double startY, double endX, double endY);
        
        void setinitialCell(int desiredinitialCell) {
            initialCell = desiredinitialCell;
        }
        void setinitialAzimuth(double desiredinitialAzimuth) {
            initialAzimuth = desiredinitialAzimuth;
        }
        void setdistibutionWeight(double desireddistibutionWeight) {
            distibutionWeight = desireddistibutionWeight;
        }
        void setazimuthWeight(double desiredazimuthWeight) {
            azimuthWeight = desiredazimuthWeight;
        }
        void setdistanceWeight(double desireddistanceWeight) {
            distanceWeight = desireddistanceWeight;
        }
        #if SEARCHGRID_USEOPP_OMPL
        void setmaxPlaningTime(double desiredmaxPlaningTime) {
            maxPlaningTime = desiredmaxPlaningTime;
        }
        #endif
        private:
            SearchGrid *grid;
            int initialCell = 1;
            double initialAzimuth = 0.0;
            double distibutionWeight = 1;
            double azimuthWeight = 0.001;
            double distanceWeight = 0.0005;
            #if SEARCHGRID_USEOPP_OMPL
            double maxPlaningTime = 2.0;
            #endif
    };
}
#endif