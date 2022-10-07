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
        SearchGrid(sqlite3 *db);
        ~SearchGrid();
        /// @brief 
        /// @param minX 
        /// @param minY 
        /// @param maxX 
        /// @param maxY 
        /// @param gridsize 
        /// @param gridType 
        /// @param SRID 
        void createGrid(double minX, double minY, double maxX, double maxY, unsigned gridsize, gridtypes_t gridType = SQUARE);
        /// @brief 
        /// @param minX 
        /// @param minY 
        /// @param maxX 
        /// @param maxY 
        /// @return 
        bool setGridWeights(double minX, double minY, double maxX, double maxY);
        /// @brief 
        void deleteGrid();
        /// @brief 
        /// @param startCell 
        /// @return 
        std::vector<int> calculateSearchPath(int startCell);
        /// @brief 
        /// @param startCell 
        /// @return 
        std::vector<int> calculateSearchPathAzimuth(int startCell);
        /// @brief 
        /// @param  
        /// @return 
        std::vector<std::pair<double, double>> locationsFromCells(std::vector<int>, unsigned outputSRID = 4326);
        /// @brief 
        /// @param  
        /// @param treeName 
        /// @param tree 
        void pathToDBTree(std::vector<std::pair<double, double>>, std::string treeName, ENCGIS::DBTree* tree);
        /// @brief 
        /// @param cell 
        /// @return 
        std::pair<double,double> getCellLocation(int cell, unsigned outputSRID = 4326);
        /// @brief 
        /// @param X 
        /// @param Y 
        /// @return 
        int getClosestCell(double X, double Y);
        /// @brief 
        /// @param cell 
        /// @return 
        int getClosestUnsearchedCell(int cell);
        /// @brief 
        /// @param cell 
        /// @return 
        int getLocalOptimalNeighbour(int cell);
        /// @brief 
        /// @param cell 
        /// @return 
        int getLocalOptimalNeighbourAzimuth(int cell);
        /// @brief 
        /// @param cell1 
        /// @param cell2 
        /// @return 
        double getAzimuth(int cell1, int cell2);
        /// @brief 
        /// @param cell 
        /// @param weight 
        void setCellWeight(int cell, int weight);
      private:
        sqlite3* m_db;
        std::string landTable;
        /// dbGridTable The name of the grid layer/table in the Spatialite database
        std::string dbGridTable;
        unsigned SRID;

    };
}

#endif //ENCGIS_SEARCHGRID_HPP_INCLUDED