//***************************************************************************
// Copyright 2020-2021 Norwegian University of Science and Technology       *
// Department of Engineering Technology                                     *
//***************************************************************************
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

#ifndef ENCGIS_SEARCHGRID_HPP_INCLUDED
#define ENCGIS_SEARCHGRID_HPP_INCLUDED


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

        /// @brief 
        /// @param db 
        /// @param in_landTable 
        /// @param in_dbGridTable 
        /// @param in_SRID 
        SearchGrid(ENCGIS::DBconnection *db, std::string in_landTable = "innavigable", std::string in_dbGridTable = "searchgrid", unsigned in_SRID = 32632);
        /// @brief 
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

        unsigned getSRID() {
          return SRID;
        }

        std::string getdbGridTable() {
          return dbGridTable;
        }

        ENCGIS::DBconnection* getDBconnection() {
          return m_con;
        }
      private:
        /// @brief 
        ENCGIS::DBconnection* m_con;
        /// @brief Table of POLYGON geometry considered as obstacle
        std::string landTable;
        /// @brief The name of the grid layer/table in the Spatialite database
        std::string dbGridTable;
        /// @brief The SRID to use for the created table
        unsigned SRID;
        /// @brief 
        std::string landTabledb;
        /// @brief 
        std::string dbGridTabledb;

    };
}

#endif //ENCGIS_SEARCHGRID_HPP_INCLUDED