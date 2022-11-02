//***************************************************************************
// Copyright 2020-2021 Norwegian University of Science and Technology       *
// Department of Engineering Technology                                     *
//***************************************************************************
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

#ifndef ENCGIS_SEARCHGRID_HPP_INCLUDED
#define ENCGIS_SEARCHGRID_HPP_INCLUDED
/* TODO: 


*/

// SQLITE3 headers.
#include <sqlite3/sqlite3.h>
#include <ENCGIS/DBconnection.hpp>
#include <ENCGIS/DBTree.hpp>

#include <vector>
#include <utility>
#include <stdexcept>
namespace ENCGIS
{
    /// @brief 
    class SearchGrid {
      public:
      typedef enum
      {
        HEXAGONAL = 0,
        SQUARE = 1,
        TRIANGULAR = 2,
      } gridtypes_t;

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

        SearchGrid(ENCGIS::DBconnection *db);
        ~SearchGrid();

        /// @brief Turns gridtypes_t ENUM into string
        /// @param gridType 
        /// @return String containing written grid type according to spatialite/GEOS functions.
        std::string gridTypeToString(gridtypes_t gridType);

        /// @brief Creates a grid within the square with minXY and maxXY as the diagonal. 
        /// @param minX Lower X coordinate defined in this.SRID
        /// @param minY Lover Y coordinate defined in this.SRID
        /// @param maxX Upper X coordinate defined in this.SRID
        /// @param maxY Upper Y coordinate defined in this.SRID
        /// @param gridsize Edge length
        /// @param gridType The shape of the grid cells
        bool createGrid(double minX, double minY, double maxX, double maxY, unsigned gridsize, gridtypes_t gridType = SQUARE);

        /// @brief Creates a grid within the polygon defined in EWKTpolygon
        /// @param EKWTpolygon An extended well-known text representation of the desired area the grid should cover.
        /// @param gridsize Edge length
        /// @param gridType The shape of the grid cells
        bool createGrid(const std::string &EWKTpolygon, unsigned gridsize, gridtypes_t gridType = SQUARE);

        /// @brief Set grid weights as distance to landTable
        /// @return TODO: currently unused
        bool setGridWeightsFromLandDistance();

        /// @brief Delete the grid from the database (The tables this.dbGridTable and dbGridTable + raw in the db opened by m_db)
        void deleteGrid();

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

        /// @brief Get locations from a vector of cell numbers. Returns in same order as input.
        /// @param cells The cells to get locations from
        /// @param outputSRID Desired SRID of the returned cells
        /// @return A vector with the (X, Y) coordinates of cells with ids given in cells, specified in outputSRID
        std::vector<std::pair<double, double>> locationsFromCells(const std::vector<int> &cells, unsigned outputSRID = 4326);

        /// @brief Finds the maximum and minimum weights, and calculates weights in the intervall [0,1]
        /// @param invert Invert the resulting weights (Ex. For distance.)
        void normalizeWeights(bool invert);

        /// @brief Get the coordinates of a cell in the specified SRID
        /// @param cell The cell in the grid for which the coordinates are returned
        /// @param outputSRID The desired SRID of the location
        /// @return The (X, Y) coordinates of the cell in specified in SRID
        std::pair<double,double> getCellLocation(int cell, unsigned outputSRID = 4326);

        /// @brief Find the closest cell to a given location
        /// @param X Coordinate in this.SRID
        /// @param Y Coordinate in this.SRID
        /// @return cell ID of closest cell to XY
        int getClosestCell(double X, double Y);

        /// @brief Finds the closest cell that has yet to be visited (has weight that is not -1).
        /// @param cell Id of the cell to search from
        /// @return The closest unvisited/unsearched cell id.
        int getClosestUnsearchedCell(int cell);

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

        /// @brief Get the azimuth/angle of the span between two cells
        /// @param cell1 
        /// @param cell2 
        /// @return The azimuth/angle between two cells
        double getAzimuth(int cell1, int cell2);

        /// @brief Set the weight of a single cell
        /// @param cell The cell whose weight will be set
        /// @param weight The weight which the cell is to have
        /// @return 
        bool setCellWeight(int cell, int weight);

        /// @brief Iterates through the cells of a path, removing cells that do not cause a change in azimuth.
        /// This is results in a lossless simplification, potentioaly reducing the number of waypoints required to form a path.
        /// @param cells The cell sequence representing the path
        /// @param acceptedAzimuthDeviation The azimuth deviation considered low enough to remove a cell
        /// @return A cell representation of the path with redundant cell specifications removed
        std::vector<int> removeRedundantCells(const std::vector<int> &cells, double acceptedAzimuthDeviation = 0.00000001);

        int getGlobalOptimalCell(int cell, double azimuth, double azimuthWeight = 0.001, double distanceWeight = 0.0003);
        std::vector<int> calculateSearchPathGlobal(int startCell, double initialAzimuth, double azimuthWeight = 0.1, double distanceWeight = 0.1);
      private:
        ENCGIS::DBconnection* m_con;
        /// @brief Table of POLYGON geometry considered as obstacle
        std::string landTable;
        /// @brief The name of the grid layer/table in the Spatialite database
        std::string dbGridTable;
        /// @brief The SRID to use for the created table
        unsigned SRID;

    };
}

#endif //ENCGIS_SEARCHGRID_HPP_INCLUDED