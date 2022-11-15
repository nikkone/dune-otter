#include "SearchGrid.hpp"
#if SEARCHGRID_USEOPP_OMPL
#include <OMPL/OMPLfunctions.hpp>
#endif

#include <iostream>
namespace ENCGIS {
    SearchGrid::SearchGrid(ENCGIS::DBconnection *db, std::string in_landTable, std::string in_dbGridTable, unsigned in_SRID) {
        m_con = db;
        landTable = in_landTable;
        dbGridTable = in_dbGridTable;
        SRID = in_SRID;
        landTabledb = "db1";
        dbGridTabledb = "main";
    }
    
    SearchGrid::~SearchGrid() {

    }

    std::string SearchGrid::gridTypeToString(gridtypes_t gridType) {
        std::string geometry;
        switch(gridType) {
            case HEXAGONAL:
                geometry = "Hexagonal";
                break;
            case SQUARE:
                geometry = "Square";
                break;
            case TRIANGULAR:
                geometry = "Triangular";
                break;
            default:
                geometry = "Square";
        }
        return geometry;
    }

    bool SearchGrid::createGrid(double minX, double minY, double maxX, double maxY, unsigned gridsize, gridtypes_t gridType){
        std::string EWKTsquare = "SRID=" + std::to_string(SRID) + ";POLYGON((" + std::to_string(minX) + " " + std::to_string(minY) + "," + std::to_string(minX) + " " + std::to_string(maxY) + ","
        "" + std::to_string(maxX) + " " + std::to_string(maxY) + "," + std::to_string(maxX) + " " + std::to_string(minY) + "," + std::to_string(minX) + " " + std::to_string(minY) + "))";

        return createGrid(EWKTsquare, gridsize, gridType);
    }

    bool SearchGrid::createGrid(const std::string &EWKTpolygon, unsigned gridsize, gridtypes_t gridType) {

        std::string create = "create table " + dbGridTable + "raw as select " + gridTypeToString(gridType) + "Grid(transform("
        "GeomFromEWKT('" + EWKTpolygon + "'), " + std::to_string(SRID) + "), " +std::to_string(gridsize)+ ") as geometry";
        
        std::string recoverMultiTable = "SELECT RecoverGeometryColumn('" + dbGridTable + "raw', 'geometry', " + std::to_string(SRID) + ", 'MULTIPOLYGON', 'XY')";
        std::string polygonFromMultipolygon = "SELECT ElementaryGeometries('" + dbGridTable + "raw', 'geometry', '" + dbGridTable + "','gid','weight') as geom FROM " + dbGridTable + "raw";

        std::string deleteLandCells = "delete from " + dbGridTable + " where gid in (select " + dbGridTable + ".gid from " + dbGridTable + ", (select geometry from " + landTabledb + "." + landTable + " where " + landTabledb + "." + landTable + ".ROWID IN ("
    "SELECT ROWID FROM SpatialIndex "
    "WHERE f_table_name = 'DB=" + landTabledb + "." + landTable + "' AND "
        "search_frame = (select GetLayerExtent('" + dbGridTable + "')))) as land where intersects(" + dbGridTable + ".geometry, land.geometry))";
        std::cout << deleteLandCells << std::endl;
        
        m_con->runNoOutputQuery(create);
        m_con->runNoOutputQuery(recoverMultiTable);
        m_con->runNoOutputQuery(polygonFromMultipolygon);
        return m_con->runNoOutputQuery(deleteLandCells);
    }

    bool SearchGrid::setGridWeightsFromLandDistance() {
        /*std::string weights = "update " + dbGridTable + " set weight = distweight from ("
        "select gid as gidsel, min(distance(centroid(" + dbGridTable + ".geometry), i)) as distweight, " + dbGridTable + ".geometry as geom from " + dbGridTable + ",(select geometry as i from " + landTabledb + "." + landTable + " WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = 'DB=" + landTabledb + "." + landTable + "' AND "
        "search_frame = (select GetLayerExtent('" + dbGridTable + "')))) group by gid) where gid = gidsel";*/
        std::string weights = "update " + dbGridTable + " set weight = distweight from ("
        "select g,gid as gidsel, min(distance(centroid(" + dbGridTable + ".geometry), i)) as distweight, " + dbGridTable + ".geometry as geom from " + dbGridTable + ",(select t.'group' as g, geometry as i from " + landTabledb + "." + landTable + " as t WHERE"
        " ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = 'DB=" + landTabledb + "." + landTable + "' AND "
        "search_frame = (select GetLayerExtent('" + dbGridTable + "')))) group by gid,g ) where gid = gidsel and g = 13";
        std::cout << weights << std::endl;
        return m_con->runNoOutputQuery(weights);
    }

    void SearchGrid::deleteGrid() {
        std::string deleteQuery = "select DropTable(NULL, '" + dbGridTable + "', TRUE)";
        std::string deleteQueryraw = "select DropTable(NULL, '" + dbGridTable + "raw', TRUE)";

        m_con->runNoOutputQuery(deleteQuery);
        m_con->runNoOutputQuery(deleteQueryraw);
    }

#if SEARCHGRID_USEOPP_OMPL
    std::vector<std::pair<double, double>>SearchGrid::calculateSearchPath(int startCell, og::SimpleSetup &setup) {
        std::vector<int> cells;
        std::vector<std::pair<double, double>> cells_pos;
        unsigned outputSRID = SRID;
        int cell = startCell;
        // Greedy algorithm
        while(cell != 0) {
            cells.push_back(cell);
            cells_pos.push_back(getCellLocation(cell, outputSRID));
            setCellWeight(cell,-1);
            cell = getLocalOptimalNeighbour(cell);
            if(cell == 0) {
                cell = getClosestUnsearchedCell(cells.back());
                if(cell != 0) {
                    auto start = getCellLocation(cells.back(),SRID);
                    auto end = getCellLocation(cell,SRID);
                    double maxPlaningTime = 2.0;
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
        /*for(auto i = cells_pos.begin(); i < cells_pos.end();i++) {
            printf("%f, %f\n", i->first, i->second);
        }*/
        return cells_pos;
    }
#endif
    std::vector<int> SearchGrid::calculateSearchPath(int startCell) {
        std::vector<int> cells;
        int cell = startCell;
        // Greedy algorithm
        while(cell != 0) {
            cells.push_back(cell);
            setCellWeight(cell,-1);
            cell = getLocalOptimalNeighbour(cell);
            if(cell == 0) {
                cell = getClosestUnsearchedCell(cells.back());
            }
            //inf("Cell: %d", cell);
        }
        return cells;
    }

    std::vector<int> SearchGrid::calculateSearchPathAzimuth(int startCell, double initialAzimuth, double azimuthWeight) {
        std::vector<int> cells;
        int cell = startCell;
        // Greedy algorithm with azimuth weights
        double azimuth = initialAzimuth;
        while(cell != 0) {
            cells.push_back(cell);
            setCellWeight(cell,-1);
            cell = getLocalOptimalNeighbourAzimuth(cell, azimuth, azimuthWeight);
            if(cell == 0) {
                cell = getClosestUnsearchedCell(cells.back());
            }
        }
        return cells;
    }

    std::vector<int> SearchGrid::calculateSearchPathDistance(int startCell) {
        std::vector<int> cells;
        int cell = startCell;
        // Greedy algorithm
        while(cell != 0) {
            cells.push_back(cell);
            setCellWeight(cell,-1);
            cell = getDistanceOptimalNextCell(cell);
            if(cell == 0) {
                cell = getClosestUnsearchedCell(cells.back());
            }
            //inf("Cell: %d", cell);
        }
        return cells;
    }

    std::vector<std::pair<double, double>> SearchGrid::locationsFromCells(const std::vector<int> &cells, unsigned outputSRID) {
        std::vector<std::pair<double, double>> out;
        for(auto i = cells.begin(); i < cells.end();i++) {
            out.push_back(getCellLocation(*i, outputSRID));
        }
        return out;
    }

    void SearchGrid::normalizeWeights(bool invert) {
        // Find max/min weight
        std::string maxmin = "select min(weight), max(weight) from " + dbGridTable;
        int errors = 0;
        sqlite3_stmt* m_handle;

        if (sqlite3_prepare_v2(m_con->db, maxmin.c_str(), maxmin.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        int m_idx = 0;
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        double min = sqlite3_column_double(m_handle, m_idx++);
        double max = sqlite3_column_double(m_handle, m_idx++);
        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);

        // Recalculate weights
        std::string recalculateWeights;
        if(invert) {
            recalculateWeights = "update " + dbGridTable + " set weight = 1 + (" + std::to_string(min) + " - weight)/" + std::to_string(max-min) + "";
        } else {
            recalculateWeights = "update " + dbGridTable + " set weight = (weight - " + std::to_string(min) + ")/" + std::to_string(max-min) + "";
        }

        if (sqlite3_prepare_v2(m_con->db, recalculateWeights.c_str(), recalculateWeights.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);
    }

    std::pair<double,double> SearchGrid::getCellLocation(int cell, unsigned outputSRID) {
        std::string query = "select X(center), Y(center) from (select transform(centroid(geometry), " + std::to_string(outputSRID) + ") as center from " + dbGridTable + " where gid = " + std::to_string(cell) + ")";
        int errors = 0;
        sqlite3_stmt* m_handle;

        if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        int m_idx = 0;
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        double X = sqlite3_column_double(m_handle, m_idx++);
        double Y = sqlite3_column_double(m_handle, m_idx++);
        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);

        return std::pair<double,double>(X, Y);
    }
    
    int SearchGrid::getClosestCell(double X, double Y) {
        std::string query = "select gid from ("
                            "select gid, min(distance(geometry, makepoint(" + std::to_string(X) + "," + std::to_string(Y) + ", 32632))) from " + dbGridTable + ")";
        int errors = 0;
        sqlite3_stmt* m_handle;

        if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        int m_idx = 0;
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        int value = sqlite3_column_int(m_handle, m_idx++);
        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);

        return value;
    }
    
    int SearchGrid::getClosestUnsearchedCell(int cell) {
        std::string query = "select gid from ("
                            "select gid, min(distance(geometry, (select geometry from " + dbGridTable + " where gid = " + std::to_string(cell) + "))) from " + dbGridTable + " where weight > 0)";
        int errors = 0;
        sqlite3_stmt* m_handle;

        if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        int m_idx = 0;
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        int value = sqlite3_column_int(m_handle, m_idx++);
        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);

        return value;
    }
    
    int SearchGrid::getLocalOptimalNeighbour(int cell) {
        std::string query = "select gid from (select max(weight) as mw,gid from " + dbGridTable + " where weight > 0 and st_touches(geometry, (select geometry from " + dbGridTable + " where gid = " + std::to_string(cell) + ")))";
        //std::cout << query << std::endl;
        int errors = 0;
        sqlite3_stmt* m_handle;

        if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        int m_idx = 0;
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        int value = sqlite3_column_int(m_handle, m_idx++);
        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);

        return value;
    }

    int SearchGrid::getLocalOptimalNeighbourAzimuth(int cell, double &azimuth, double azimuthWeight) {
        std::string query = "select gid, azimuth from ("
        "select gid, azimuth, max(weight - " + std::to_string(azimuthWeight) + "*(abs(" + std::to_string(azimuth) + " - azimuth)/(2*PI()))  ) from ("
        "select weight,gid,azimuth(centroid(geometry), (select centroid(geometry) from " + dbGridTable + " where gid = " + std::to_string(cell) + ")) as azimuth"
        " from " + dbGridTable + " where weight > 0 and st_touches(geometry, (select geometry from " + dbGridTable + " where gid = " + std::to_string(cell) + "))"
        "))";

        
        //std::string query = "select gid from (select max(weight) as mw,gid from " + dbGridTable + " where weight > 0 and st_touches(geometry, (select geometry from " + dbGridTable + " where gid = " + std::to_string(cell) + ")))";
        int errors = 0;
        sqlite3_stmt* m_handle;

        if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        int m_idx = 0;
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        int value = sqlite3_column_int(m_handle, m_idx++);
        azimuth = sqlite3_column_double(m_handle, m_idx++);

        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);

        return value;
    }
    
    int SearchGrid::getDistanceOptimalNextCell(int cell) {
        double distanceWeight = 2.0;
        double maxDistance = 1850;
        std::string query = "select *, max(weight - " + std::to_string(distanceWeight) + "*distance(geometry, (select geometry from " + dbGridTable + " where gid = " + std::to_string(cell) + "))/" + std::to_string(maxDistance) + ") from " + dbGridTable + " where weight > 0";
        int errors = 0;
        sqlite3_stmt* m_handle;

        if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        int m_idx = 0;
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        int value = sqlite3_column_int(m_handle, m_idx++);

        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);

        return value;

    }

    double SearchGrid::getAzimuth(int cell1, int cell2) {
        std::string query = " select azimuth((select centroid(geometry) from " + dbGridTable + " where gid = " + std::to_string(cell1) + "), (select centroid(geometry) from " + dbGridTable + " where gid = " + std::to_string(cell2) + "))";
        int errors = 0;
        sqlite3_stmt* m_handle;

        if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        int m_idx = 0;
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        double azimuth = sqlite3_column_double(m_handle, m_idx++);
        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);
        return azimuth;
    }

    bool SearchGrid::setCellWeight(int cell, int weight) {
        std::string query = "update " + dbGridTable + " set weight = " + std::to_string(weight) + " where gid = " + std::to_string(cell) + "";
        return m_con->runNoOutputQuery(query);
    }

    std::vector<int> SearchGrid::removeRedundantCells(const std::vector<int> &cells, double acceptedAzimuthDeviation) {
        std::vector<int> resultingCells;
        resultingCells.push_back(*(cells.begin()));

        double previousAzimuth = getAzimuth(cells[0], cells[1]);
        for(auto i = cells.begin()+1; i < cells.end()-1;i++) {
            double currentAzimuth = getAzimuth(*i, *(i+1));
            //std::cout << previousAzimuth << " - "<< currentAzimuth << " - " << std::abs(previousAzimuth - currentAzimuth) << " - " << *i << " - "  << *(i+1) <<std::endl;
            if(std::abs(previousAzimuth - currentAzimuth) > acceptedAzimuthDeviation) {
                resultingCells.push_back(*i);
                previousAzimuth = currentAzimuth;
            }
        }

        resultingCells.push_back(cells.back());
        return resultingCells;
    }

    int SearchGrid::getGlobalOptimalCell(int cell, double azimuth, double azimuthWeight, double distanceWeight) {
        std::string query = "select gid from (select gid,"
        "max(weight"
        "- " + std::to_string(azimuthWeight) + "*ABS( " + std::to_string(azimuth) + "-azimuth((select centroid(geometry) from " + dbGridTable + " where gid = " + std::to_string(cell) + "), (select centroid(geometry) from " + dbGridTable + " where gid = sg.gid)))"
        "- " + std::to_string(distanceWeight) + "*distance((select centroid(geometry) from " + dbGridTable + " where gid = " + std::to_string(cell) + "), (select centroid(geometry) from " + dbGridTable + " where gid = sg.gid))) as wgt "
        "from " + dbGridTable + " as sg where weight between -0.1 and 1.1)";
        //std::cout << query << std::endl;
        int errors = 0;
        sqlite3_stmt* m_handle;

        if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
        {
            errors++;
        }
        int m_idx = 0;
        // Execute
        /*int rc = */sqlite3_step(m_handle);
        int value = sqlite3_column_int(m_handle, m_idx++);
        // Teardown
        if (m_handle)
            sqlite3_finalize(m_handle);

        return value;
    }
    
    std::vector<int> SearchGrid::calculateSearchPathGlobal(int startCell, double initialAzimuth, double azimuthWeight, double distanceWeight)
    {
        /*
        setCellWeight(startCell, 2);
        int cellNum = 5;
        for(int i = 0; i < cellNum; i++) {

            setCellWeight(startCell, i+2);
        }*/
        std::vector<int> cells;
        int cell = startCell;
        // Greedy algorithm with azimuth weights
        double azimuth = initialAzimuth;
        while(cell != 0) {
            cells.push_back(cell);
            setCellWeight(cell,-1);
            cell = getGlobalOptimalCell(cell, azimuth, azimuthWeight, distanceWeight);
            std::cout << cell << std::endl;
        }
        return cells;

    }

    #if SEARCHGRID_USEOPP_OMPL
    std::vector<std::pair<double, double>> SearchGrid::calculateSearchPathGlobal(int startCell, og::SimpleSetup &setup, double initialAzimuth, double azimuthWeight, double distanceWeight)
    {
        std::vector<std::pair<double, double>> cells_pos;
        std::vector<int> cells;
        int cell = startCell;
        // Greedy algorithm with azimuth weights
        double azimuth = initialAzimuth;
        while(cell != 0) {
            cells.push_back(cell);
            setCellWeight(cell,-1);
            cell = getGlobalOptimalCell(cell, azimuth, azimuthWeight, distanceWeight);
            if(cell == 0) {
                cells_pos.push_back(getCellLocation(cells.back(),SRID));
                break;
            }
            auto start = getCellLocation(cells.back(),SRID);
            auto end = getCellLocation(cell,SRID);
            double maxPlaningTime = 2.0;
            OMPLintegrationENCGIS::setStartAndGoalStates(setup, start.first, start.second, end.first, end.second);
            og::PathGeometric states = OMPLintegrationENCGIS::findPath(setup, maxPlaningTime, OMPLintegrationENCGIS::configurations_t::C_KBIT);
            if (states.getStateCount()) {
                auto planVec = OMPLforDUNE::pathToVector(states);
                cells_pos.insert( cells_pos.end(), planVec.begin(), planVec.end()-1 );
            } else {
                std::cout << "Error finding path from: " << start.first << "," << start.second << " to " << end.first << "," << end.second << std::endl;
            }
        }
        return cells_pos;
    }
    #endif
}