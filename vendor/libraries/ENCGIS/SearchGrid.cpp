#include "SearchGrid.hpp"
#include <iostream>
namespace ENCGIS {
    SearchGrid::SearchGrid(ENCGIS::DBconnection *db) {
        m_con = db;
        landTable = "innavigable";
        dbGridTable = "searchgrid";
        SRID = 32632;
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

        std::string deleteLandCells = "delete from " + dbGridTable + " where gid in (select " + dbGridTable + ".gid from " + dbGridTable + ", (select geometry from " + landTable + " where " + landTable + ".ROWID IN ("
    "SELECT ROWID FROM SpatialIndex "
    "WHERE f_table_name = '" + landTable + "' AND "
        "search_frame = (select GetLayerExtent('" + dbGridTable + "')))) as land where intersects(" + dbGridTable + ".geometry, land.geometry))";
        //std::cout << create << std::endl;
        
        m_con->runNoOutputQuery(create);
        m_con->runNoOutputQuery(recoverMultiTable);
        m_con->runNoOutputQuery(polygonFromMultipolygon);
        return m_con->runNoOutputQuery(deleteLandCells);
    }

    bool SearchGrid::setGridWeightsFromLandDistance() {
        std::string weights = "update " + dbGridTable + " set weight = distweight from ("
        "select gid as gidsel, min(distance(centroid(" + dbGridTable + ".geometry), i)) as distweight, " + dbGridTable + ".geometry as geom from " + dbGridTable + ",(select geometry as i from " + landTable + " WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + landTable + "' AND "
        "search_frame = (select GetLayerExtent('" + dbGridTable + "')))) group by gid) where gid = gidsel";

        return m_con->runNoOutputQuery(weights);
    }

    void SearchGrid::deleteGrid() {
        std::string deleteQuery = "select DropTable(NULL, '" + dbGridTable + "', TRUE)";
        std::string deleteQueryraw = "select DropTable(NULL, '" + dbGridTable + "raw', TRUE)";

        m_con->runNoOutputQuery(deleteQuery);
        m_con->runNoOutputQuery(deleteQueryraw);
    }
    
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
}