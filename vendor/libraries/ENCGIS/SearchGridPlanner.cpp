#include "SearchGridPlanner.hpp"
#if SEARCHGRID_USEOPP_OMPL
#include <OMPL/OMPLfunctions.hpp>
#endif

#include <iostream>
namespace ENCGIS {
#if SEARCHGRID_USEOPP_OMPL
    std::vector<std::pair<double, double>>SearchGridPlanner::calculateSearchPath(og::SimpleSetup &setup) {
        std::vector<int> cells;
        std::vector<std::pair<double, double>> cells_pos;
        unsigned outputSRID = grid->getSRID();
        int cell = initialCell;
        // Greedy algorithm
        while(cell != 0) {
            cells.push_back(cell);
            cells_pos.push_back(grid->getCellLocation(cell, outputSRID));
            grid->setCellMetric(cell,-1);
            cell = getLocalOptimalNeighbour(cell);
            if(cell == 0) {
                cell = grid->getClosestUnsearchedCell(cells.back());
                if(cell != 0) {
                    auto start = grid->getCellLocation(cells.back(),grid->getSRID());
                    auto end = grid->getCellLocation(cell,grid->getSRID());
                    //std::cout << start.second << "," << start.first << "," << end.second << "," << end.first << std::endl;
                    //std::cout << start.first << "," << start.second << "," << end.first << "," << end.second << std::endl;

                    //OMPLintegrationENCGIS::setStartAndGoalStates(setup, start.second,start.first, end.second,end.first);
                    OMPLintegrationENCGIS::setStartAndGoalStates(setup, start.first, start.second, end.first, end.second);
                    og::PathGeometric states = OMPLintegrationENCGIS::findPath(setup, maxPlaningTime, OMPLintegrationENCGIS::configurations_t::C_KBIT);
                        if (states.getStateCount()) {
                            auto planVec = OMPLforDUNE::pathToVector(states);
                            cells_pos.insert( cells_pos.end(), planVec.begin()+1, planVec.end()-1 );
                        }
                }
            }
        }
        return cells_pos;
    }

    std::vector<std::pair<double, double>>SearchGridPlanner::calculateRandomLocalSearchPath(og::SimpleSetup &setup) {
        std::vector<int> cells;
        std::vector<std::pair<double, double>> cells_pos;
        unsigned outputSRID = grid->getSRID();
        int cell = initialCell;
        // Greedy algorithm
        while(cell != 0) {
            cells.push_back(cell);
            cells_pos.push_back(grid->getCellLocation(cell, outputSRID));
            grid->setCellMetric(cell,-1);
            cell = getLocalRandomNeighbour(cell);
            if(cell == 0) {
                cell = grid->getClosestUnsearchedCell(cells.back());
                if(cell != 0) {
                    auto start = grid->getCellLocation(cells.back(),grid->getSRID());
                    auto end = grid->getCellLocation(cell,grid->getSRID());
                    //std::cout << start.second << "," << start.first << "," << end.second << "," << end.first << std::endl;
                    //std::cout << start.first << "," << start.second << "," << end.first << "," << end.second << std::endl;

                    //OMPLintegrationENCGIS::setStartAndGoalStates(setup, start.second,start.first, end.second,end.first);
                    OMPLintegrationENCGIS::setStartAndGoalStates(setup, start.first, start.second, end.first, end.second);
                    og::PathGeometric states = OMPLintegrationENCGIS::findPath(setup, maxPlaningTime, OMPLintegrationENCGIS::configurations_t::C_KBIT);
                        if (states.getStateCount()) {
                            auto planVec = OMPLforDUNE::pathToVector(states);
                            cells_pos.insert( cells_pos.end(), planVec.begin()+1, planVec.end()-1 );
                        }
                }
            }
        }
        return cells_pos;
    }
#endif
    std::vector<int> SearchGridPlanner::calculateSearchPath() {
        std::vector<int> cells;
        int cell = initialCell;
        // Greedy algorithm
        while(cell != 0) {
            cells.push_back(cell);
            grid->setCellMetric(cell,-1);
            cell = getLocalOptimalNeighbour(cell);
            if(cell == 0) {
                cell = grid->getClosestUnsearchedCell(cells.back());
            }
            //inf("Cell: %d", cell);
        }
        return cells;
    }

    std::vector<int> SearchGridPlanner::calculateSearchPathAzimuth() {
        std::vector<int> cells;
        int cell = initialCell;
        // Greedy algorithm with azimuth weights
        double azimuth = initialAzimuth;
        while(cell != 0) {
            cells.push_back(cell);
            grid->setCellMetric(cell,-1);
            cell = getLocalOptimalNeighbourAzimuth(cell, azimuth);
            if(cell == 0) {
                cell = grid->getClosestUnsearchedCell(cells.back());
            }
        }
        return cells;
    }

    std::vector<int> SearchGridPlanner::calculateSearchPathDistance() {
        std::vector<int> cells;
        int cell = initialCell;
        // Greedy algorithm
        while(cell != 0) {
            cells.push_back(cell);
            grid->setCellMetric(cell,-1);
            cell = getDistanceOptimalNextCell(cell);
            if(cell == 0) {
                cell = grid->getClosestUnsearchedCell(cells.back());
            }
            //inf("Cell: %d", cell);
        }
        return cells;
    }

    int SearchGridPlanner::getLocalOptimalNeighbour(int cell, std::string metric) {
        std::string query = "select gid from (select max(" + metric + ") as mw,gid from " + grid->getdbGridTable() + " where " + metric + " >= 0 and st_touches(geometry, (select geometry from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + ")))";
        //std::cout << query << std::endl;
        sqlite3_stmt* m_handle = nullptr;

        if (sqlite3_prepare_v2(grid->getDBconnection()->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
            if(sqlite3_step(m_handle) == SQLITE_ROW) {
              int value = sqlite3_column_int(m_handle, 0);
              sqlite3_finalize(m_handle);
              return value;
            }
        }
        return -1;
    }

    int SearchGridPlanner::getLocalOptimalNeighbourAzimuth(int cell, double &azimuth, std::string metric) {
        std::string query = "select gid, azimuth from ("
        "select gid, azimuth, max(" + metric + " - " + std::to_string(azimuthWeight) + "*(abs(" + std::to_string(azimuth) + " - azimuth)/(2*PI()))  ) from ("
        "select " + metric + ",gid,azimuth(center, (select center from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + ")) as azimuth"
        " from " + grid->getdbGridTable() + " where " + metric + " > 0 and st_touches(geometry, (select geometry from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "))"
        "))";

        //std::string query = "select gid from (select max(" + metric + ") as mw,gid from " + grid->getdbGridTable() + " where " + metric + " > 0 and st_touches(geometry, (select geometry from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + ")))";
        sqlite3_stmt* m_handle = nullptr;
        int value = 0;
        if (sqlite3_prepare_v2(grid->getDBconnection()->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
          if(sqlite3_step(m_handle) == SQLITE_ROW) {
            value = sqlite3_column_int(m_handle, 0);
            azimuth = sqlite3_column_double(m_handle, 1);
          }
        }

        sqlite3_finalize(m_handle);
        return value;
    }
    
    int SearchGridPlanner::getDistanceOptimalNextCell(int cell, std::string metric) {
      double maxDistance = 1850;
      std::string query = "select *, max(" + metric + " - " + std::to_string(distanceWeight) + "*distance(geometry, (select geometry from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "))/" + std::to_string(maxDistance) + ") from " + grid->getdbGridTable() + " where " + metric + " > 0";
      sqlite3_stmt* m_handle = nullptr;
      int value = 0;
      if (sqlite3_prepare_v2(grid->getDBconnection()->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          value = sqlite3_column_int(m_handle, 0);
        }
      }
      sqlite3_finalize(m_handle);
      return value;
    }
    std::vector<int> SearchGridPlanner::removeRedundantCells(const std::vector<int> &cells, double acceptedAzimuthDeviation) {
        std::vector<int> resultingCells;
        resultingCells.push_back(*(cells.begin()));

        double previousAzimuth = grid->getAzimuth(cells[0], cells[1]);
        for(auto i = cells.begin()+1; i < cells.end()-1;i++) {
            double currentAzimuth = grid->getAzimuth(*i, *(i+1));
            //std::cout << previousAzimuth << " - "<< currentAzimuth << " - " << std::abs(previousAzimuth - currentAzimuth) << " - " << *i << " - "  << *(i+1) <<std::endl;
            if(std::abs(previousAzimuth - currentAzimuth) > acceptedAzimuthDeviation) {
                resultingCells.push_back(*i);
                previousAzimuth = currentAzimuth;
            }
        }
        resultingCells.push_back(cells.back());
        return resultingCells;
    }

    int SearchGridPlanner::getLocalRandomNeighbour(int cell, std::string metric) {
        //std::string query = "select *, max(" + metric + " - " + std::to_string(distanceWeight) + "*distance(geometry, (select geometry from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "))/" + std::to_string(maxDistance) + ") from " + grid->getdbGridTable() + " where " + metric + " > 0";
        std::string query = "select gid from ("
            "	select "
            "		gid from " + grid->getdbGridTable() + ""
            "	where "
            "		" + metric + " >= 0 and "
            "		st_touches(geometry, (select geometry from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "))"
            ") ORDER BY RANDOM() LIMIT 1;";
        sqlite3_stmt* m_handle = nullptr;
        int value = 0;
        if (sqlite3_prepare_v2(grid->getDBconnection()->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
          if(sqlite3_step(m_handle) == SQLITE_ROW) {
            value = sqlite3_column_int(m_handle, 0);
          }
        }
        sqlite3_finalize(m_handle);
        return value;
    }

    int SearchGridPlanner::getGlobalRandomCell(std::string metric) {
      //std::string query = "select *, max(" + metric + " - " + std::to_string(distanceWeight) + "*distance(geometry, (select geometry from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "))/" + std::to_string(maxDistance) + ") from " + grid->getdbGridTable() + " where " + metric + " > 0";
      std::string query = "select gid from " + grid->getdbGridTable() + " where " + metric + " >= 0 ORDER BY RANDOM() LIMIT 1;";
      sqlite3_stmt* m_handle = nullptr;
      int value = 0;
      if (sqlite3_prepare_v2(grid->getDBconnection()->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          value = sqlite3_column_int(m_handle, 0);
        }
      }
      sqlite3_finalize(m_handle);
      return value;
    }


    int SearchGridPlanner::getGlobalOptimalCell(int cell, double azimuth, std::string metric) {
/*
        std::string query = "select gid from (select gid,"
        "max(" + metric + ""
        "- " + std::to_string(azimuthWeight) + "*ABS( " + std::to_string(azimuth) + "-azimuth((select center from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "), (select center from " + grid->getdbGridTable() + " where gid = sg.gid)))"
        "- " + std::to_string(distanceWeight) + "*distance((select center from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "), (select center from " + grid->getdbGridTable() + " where gid = sg.gid))) as wgt "
        "from " + grid->getdbGridTable() + " as sg where " + metric + " between -0.1 and 1.1)";
*/
std::string query = 
"select gid, max(w-" + std::to_string(azimuthWeight) + "/pi()*(abs((a-floor(a/n)*n)-PI()))-" + std::to_string(distanceWeight) + "*d) from ("
 	"select geometry,gid,"
	"" + metric + "/(select abs(max(" + metric + ")) from " + grid->getdbGridTable() + ") as w"
	",( (" + std::to_string(azimuth) + "-azimuth((select center from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "), (select center from " + grid->getdbGridTable() + " where gid = sg.gid))) + PI())"
	"as a"
	",distance((select center from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "), (select center from " + grid->getdbGridTable() + " where gid = sg.gid))/"
	"(select max(draw) from ("
	"select distance((select center from " + grid->getdbGridTable() + " where gid = " + std::to_string(cell) + "), (select center from " + grid->getdbGridTable() + " where gid = dg.gid)) as draw "
	"from " + grid->getdbGridTable() + " as dg"
	"))"
  	"as d,"
 	"2*PI() as n "
 	"from " + grid->getdbGridTable() + " as sg where w>0.0"
")";

      //std::cout << query << std::endl;
      int value = 0;
      sqlite3_stmt* m_handle = nullptr;
      if (sqlite3_prepare_v2(grid->getDBconnection()->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          value = sqlite3_column_int(m_handle, 0);
        }
      }
      sqlite3_finalize(m_handle);
      return value;
    }
    
    std::vector<int> SearchGridPlanner::calculateSearchPathGlobal()
    {
        /*
        grid->setCellWeight(initialCell, 2);
        int cellNum = 5;
        for(int i = 0; i < cellNum; i++) {

            grid->setCellWeight(initialCell, i+2);
        }*/
        std::vector<int> cells;
        int cell = initialCell;
        // Greedy algorithm with azimuth weights
        double azimuth = initialAzimuth;
        while(cell != 0) {
            cells.push_back(cell);
            grid->setCellMetric(cell,-1);
            cell = getGlobalOptimalCell(cell, azimuth);
            std::cout << cell << std::endl;
        }
        return cells;

    }

    #if SEARCHGRID_USEOPP_OMPL
    std::vector<std::pair<double, double>> SearchGridPlanner::calculateSearchPathGlobal(og::SimpleSetup &setup)
    {
        bool intersectingVisited = false; // TODO: Make parameter
        std::vector<std::pair<double, double>> cells_pos;
        std::vector<int> cells;
        int cell = initialCell;
        // Greedy algorithm with azimuth weights
        double azimuth = initialAzimuth;
        while(cell != 0) {
            cells.push_back(cell);
            grid->setCellMetric(cell,-1);
            cell = getGlobalOptimalCell(cell, azimuth);
            std::cout << cell << std::endl;
            if(cell == 0) {
                cells_pos.push_back(grid->getCellLocation(cells.back(),grid->getSRID()));
                break;
            }
            auto start = grid->getCellLocation(cells.back(),grid->getSRID());
            auto end = grid->getCellLocation(cell,grid->getSRID());
            OMPLintegrationENCGIS::setStartAndGoalStates(setup, start.first, start.second, end.first, end.second);
            og::PathGeometric states = OMPLintegrationENCGIS::findPath(setup, maxPlaningTime, OMPLintegrationENCGIS::configurations_t::C_KBIT);
            if (states.getStateCount()) {
                auto planVec = OMPLforDUNE::pathToVector(states);
                cells_pos.insert( cells_pos.end(), planVec.begin(), planVec.end()-1 );
                if(intersectingVisited) {
                    for(auto itr = planVec.begin();itr<planVec.end()-1;itr++) {
                        setIntersectingCellsAsVisited(itr->first,itr->second, (itr+1)->first, (itr+1)->second);
                    }
                }
            } else {
                std::cout << "Error finding path from: " << start.first << "," << start.second << " to " << end.first << "," << end.second << std::endl;
            }
        }
        return cells_pos;
    }

    std::vector<std::pair<double, double>> SearchGridPlanner::calculateRandomSearchPathGlobal(og::SimpleSetup &setup)
    {
        bool intersectingVisited = false; // TODO: Make parameter
        std::vector<std::pair<double, double>> cells_pos;
        std::vector<int> cells;
        int cell = initialCell;
        
        while(cell != 0) {
            cells.push_back(cell);
            grid->setCellMetric(cell,-1);
            cell = getGlobalRandomCell();
            std::cout << cell << std::endl;
            if(cell == 0) {
                cells_pos.push_back(grid->getCellLocation(cells.back(),grid->getSRID()));
                break;
            }
            auto start = grid->getCellLocation(cells.back(),grid->getSRID());
            auto end = grid->getCellLocation(cell,grid->getSRID());
            OMPLintegrationENCGIS::setStartAndGoalStates(setup, start.first, start.second, end.first, end.second);
            og::PathGeometric states = OMPLintegrationENCGIS::findPath(setup, maxPlaningTime, OMPLintegrationENCGIS::configurations_t::C_KBIT);
            if (states.getStateCount()) {
                auto planVec = OMPLforDUNE::pathToVector(states);
                cells_pos.insert( cells_pos.end(), planVec.begin(), planVec.end()-1 );
                if(intersectingVisited) {
                    for(auto itr = planVec.begin();itr<planVec.end()-1;itr++) {
                        setIntersectingCellsAsVisited(itr->first,itr->second, (itr+1)->first, (itr+1)->second);
                    }
                }
            } else {
                std::cout << "Error finding path from: " << start.first << "," << start.second << " to " << end.first << "," << end.second << std::endl;
            }
        }
        return cells_pos;
    }
    #endif
    void SearchGridPlanner::setIntersectingCellsAsVisited(double startX, double startY, double endX, double endY, std::string metric) {
        // Alternative: Threashold distance to centroid
        std::string query = "update " + grid->getdbGridTable() + " set " + metric + " = -1 from ("
        "select gid as gidsel from " + grid->getdbGridTable() + " where intersects(geometry, makeline(makepoint(" + std::to_string(startX) + ", " + std::to_string(startY) + ", " + std::to_string(grid->getSRID()) + "), makepoint(" + std::to_string(endX) + ", " + std::to_string(endY) + ", " + std::to_string(grid->getSRID()) + ")))"
        ") where gid = gidsel";
        //std::cout << query << std::endl;
        grid->getDBconnection()->runNoOutputQuery(query);
    }

}