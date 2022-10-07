#include "SearchGrid.hpp"
#include <iostream>
namespace ENCGIS {
        SearchGrid::SearchGrid(sqlite3 *db) {
            m_db = db;
            landTable = "innavigable";
            dbGridTable = "searchgrid";
            SRID = 32632;
        }
        SearchGrid::~SearchGrid() {

        }

        void SearchGrid::createGrid(double minX, double minY, double maxX, double maxY, unsigned gridsize, gridtypes_t gridType){
            //unsigned gridsize = 75;
            //unsigned SRID = 32632;
            //gridtypes_t geometrynr = HEXAGONAL;
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
            std::string create = "create table " + dbGridTable + "raw as select " + geometry + "Grid(BuildMbr(?1,?2,?3,?4, " + std::to_string(SRID) + "), " +std::to_string(gridsize)+ ") as geometry";
            std::string recoverMultiTable = "SELECT RecoverGeometryColumn('" + dbGridTable + "raw', 'geometry', " + std::to_string(SRID) + ", 'MULTIPOLYGON', 'XY')";
            std::string polygonFromMultipolygon = "SELECT ElementaryGeometries('" + dbGridTable + "raw', 'geometry', '" + dbGridTable + "','gid','weight') as geom FROM " + dbGridTable + "raw";

            std::string deleteLandCells = "delete from " + dbGridTable + " where gid in (select " + dbGridTable + ".gid from " + dbGridTable + ", (select geometry from " + landTable + " where " + landTable + ".ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + landTable + "' AND "
          "search_frame = (select GetLayerExtent('" + dbGridTable + "')))) as land where intersects(" + dbGridTable + ".geometry, land.geometry))";
            std::cout << create << std::endl;
            // Create sql statements
            sqlite3_stmt* m_createHandle;

            // Prepare sql statements (compile)
            sqlite3_prepare_v3(m_db, create.c_str(), -1, SQLITE_PREPARE_PERSISTENT, &m_createHandle, 0);

            // Bind values to sql statements
            {
                sqlite3_bind_double(m_createHandle,1,minX);
                sqlite3_bind_double(m_createHandle,2,minY);
                sqlite3_bind_double(m_createHandle,3,maxX);
                sqlite3_bind_double(m_createHandle,4,maxY);
            }
            // Execute sql statements
           
// Create
            if(sqlite3_step(m_createHandle) == SQLITE_ROW) {
                sqlite3_reset(m_createHandle);
            } else {
                sqlite3_reset(m_createHandle);
            }
            int errors = 0;
            sqlite3_stmt* m_handle;
            
// recoverMultiTable      
            if (sqlite3_prepare_v2(m_db, recoverMultiTable.c_str(), recoverMultiTable.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", recoverMultiTable.c_str());
            }
         
            // Execute
            /*int rc = */sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);
// polygonFromMultipolygon
            if (sqlite3_prepare_v2(m_db, polygonFromMultipolygon.c_str(), polygonFromMultipolygon.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", polygonFromMultipolygon.c_str());
            }
            // Execute
            /*rc = */sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);
// deleteLandHandle
            // Execute
            /*if(sqlite3_step(m_deleteLandHandle) == SQLITE_ROW) {
                sqlite3_reset(m_deleteLandHandle);
            } else {
                sqlite3_reset(m_deleteLandHandle);
            }*/
            if (sqlite3_prepare_v2(m_db, deleteLandCells.c_str(), deleteLandCells.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", deleteLandCells.c_str());
            }
            // Execute
            /*rc = */sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);   

        // Finalize statements
            sqlite3_finalize(m_createHandle);

        }
        bool SearchGrid::setGridWeights(double minX, double minY, double maxX, double maxY){
            std::string weights = "update " + dbGridTable + " set weight = distweight from ("
            "select gid as gidsel, min(distance(centroid(" + dbGridTable + ".geometry), i)) as distweight, " + dbGridTable + ".geometry as geom from " + dbGridTable + ",(select geometry as i from " + landTable + " WHERE ROWID IN ("
            "SELECT ROWID FROM SpatialIndex "
            "WHERE f_table_name = '" + landTable + "' AND "
            "search_frame = BuildMbr(" + std::to_string(minX) + "," + std::to_string(minY) + "," + std::to_string(maxX) + "," + std::to_string(maxY) + "))) group by gid) where gid = gidsel";

            int errors = 0;
            sqlite3_stmt* m_handle;
            if (sqlite3_prepare_v2(m_db, weights.c_str(), weights.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", weights.c_str());
            }
            // Execute
            /*int rc = */sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);

            return false;
        }
        void SearchGrid::deleteGrid(){
            std::string deleteQuery = "select DropGeoTable('" + dbGridTable + "')";
            std::string deleteQueryraw = "select DropGeoTable('" + dbGridTable + "raw')";
            int errors = 0;
            sqlite3_stmt* m_handle;

            if (sqlite3_prepare_v2(m_db, deleteQuery.c_str(), deleteQuery.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", recoverMultiTable.c_str());
            }
         
            // Execute
            /*int rc = */sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);

            if (sqlite3_prepare_v2(m_db, deleteQueryraw.c_str(), deleteQueryraw.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", recoverMultiTable.c_str());
            }
         
            // Execute
            /*rc = */sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);

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
        std::vector<int> SearchGrid::calculateSearchPathAzimuth(int startCell) {
            std::vector<int> cells;
            int cell = startCell;
            // Greedy algorithm
            while(cell != 0) {
                cells.push_back(cell);
                setCellWeight(cell,-1);
                cell = getLocalOptimalNeighbourAzimuth(cell);
                if(cell == 0) {
                    cell = getClosestUnsearchedCell(cells.back());
                }
            }
            return cells;
        }
        std::vector<std::pair<double, double>> SearchGrid::locationsFromCells(std::vector<int> cells, unsigned outputSRID) {
            std::vector<std::pair<double, double>> out;
            for(auto i = cells.begin(); i < cells.end();i++) {
                out.push_back(getCellLocation(*i, outputSRID));
            }
            return out;
        }

        void SearchGrid::pathToDBTree(std::vector<std::pair<double, double>> waypoints, std::string treeName, ENCGIS::DBTree* tree) {
            //try{
                tree->resetTree(treeName);
            //} catch(...) {
            //    err("treeName cant be reset");
            //    return;
            //}
            for(unsigned i=0;i<waypoints.size();i++) {
                if(i!=0)
                    tree->insertNode(treeName,i,waypoints[i].first, waypoints[i].second);
                else
                {
                    tree->insertNode(treeName,1,waypoints[i].first, waypoints[i].second);
                }
            }
        }

        std::pair<double,double> SearchGrid::getCellLocation(int cell, unsigned outputSRID) {
            std::string query = "select X(center), Y(center) from (select transform(centroid(geometry), " + std::to_string(outputSRID) + ") as center from " + dbGridTable + " where gid = " + std::to_string(cell) + ")";
            int errors = 0;
            sqlite3_stmt* m_handle;

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
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

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
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

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
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
            std::string query = "select gid from (select min(weight) as mw,gid from " + dbGridTable + " where weight > 0 and st_touches(geometry, (select geometry from " + dbGridTable + " where gid = " + std::to_string(cell) + ")))";
            int errors = 0;
            sqlite3_stmt* m_handle;

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
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
        int SearchGrid::getLocalOptimalNeighbourAzimuth(int cell) {
            
            std::string query = "select gid from (select min(weight) as mw,gid from " + dbGridTable + " where weight > 0 and st_touches(geometry, (select geometry from " + dbGridTable + " where gid = " + std::to_string(cell) + ")))";
            int errors = 0;
            sqlite3_stmt* m_handle;

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
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

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
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

        void SearchGrid::setCellWeight(int cell, int weight) {
            std::string query = "update " + dbGridTable + " set weight = " + std::to_string(weight) + " where gid = " + std::to_string(cell) + "";
            int errors = 0;
            sqlite3_stmt* m_handle;

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;
            }
            // Execute
            /*int rc = */sqlite3_step(m_handle);
            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);
        }
}