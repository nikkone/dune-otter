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
#include <ENCGIS/DBTree.hpp>

#include <vector>
#include <utility>
#include <stdexcept>
namespace ENCGIS
{
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
        void createGrid(double minX, double minY, double maxX, double maxY, unsigned gridsize, gridtypes_t gridType = SQUARE, std::string dbGridTable = "searchgrid", unsigned SRID = 32632);
        bool setGridWeights(std::string dbGridTable = "searchgrid");
        void deleteGrid(std::string dbGridTable = "searchgrid");
        std::vector<int> calculateSearchPath(double startCell, std::string dbGridTable = "searchgrid");
        std::vector<std::pair<double, double>> locationsFromCells(std::vector<int>, std::string dbGridTable = "searchgrid");
        void pathToDBTree(std::vector<std::pair<double, double>>, std::string treeName, ENCGIS::DBTree* tree);
        std::pair<double,double> getCellLocation(int cell, std::string dbGridTable = "searchgrid");
        int getClosestCell(double X, double Y, std::string dbGridTable = "searchgrid");
        int getClosestUnsearchedCell(int cell, std::string dbGridTable = "searchgrid");
        int getLocalOptimalNeighbour(int cell, std::string dbGridTable = "searchgrid");
        void setCellWeight(int cell, int weight, std::string dbGridTable = "searchgrid");
      private:
        sqlite3* m_db;
    };
}

#endif //ENCGIS_SEARCHGRID_HPP_INCLUDED