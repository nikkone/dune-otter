#include "SearchGrid.hpp"
#include <iostream>
namespace ENCGIS {
        SearchGrid::SearchGrid(sqlite3 *db) {
            m_db = db;

        }
        SearchGrid::~SearchGrid() {

        }

        void SearchGrid::createGrid(double minX, double minY, double maxX, double maxY, unsigned gridsize, gridtypes_t gridType, std::string dbGridTable, unsigned SRID){
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
            std::string landTable = "innavigable";

            std::string create = "create table " + dbGridTable + "raw as select " + geometry + "Grid(BuildMbr(?1,?2,?3,?4, " + std::to_string(SRID) + "), " +std::to_string(gridsize)+ ") as geometry";
            std::string recoverMultiTable = "SELECT RecoverGeometryColumn('" + dbGridTable + "raw', 'geometry', " + std::to_string(SRID) + ", 'MULTIPOLYGON', 'XY')";
            std::string polygonFromMultipolygon = "SELECT ElementaryGeometries('" + dbGridTable + "raw', 'geometry', '" + dbGridTable + "','gid','weight') as geom FROM " + dbGridTable + "raw";

            std::string deleteLandCells = "delete from " + dbGridTable + " where gid in (select " + dbGridTable + ".gid from " + dbGridTable + ", (select geometry from " + landTable + " where " + landTable + ".ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + landTable + "' AND "
          "search_frame = BuildMbr( " + std::to_string(minX) + "," + std::to_string(minY) + "," + std::to_string(maxX) + "," + std::to_string(maxY) + "))) as land where intersects(" + dbGridTable + ".geometry, land.geometry))";
            /*std::string deleteLandCells = "delete from " + dbGridTable + " where intersects(" + dbGridTable + ".geometry,"		  
                                "(select geometry from " + landTable + " where " + landTable + ".ROWID IN ("
                                    " SELECT ROWID FROM SpatialIndex "
                                    " WHERE f_table_name = '" + landTable + "' AND "
                                    "   search_frame = BuildMbr(?1,?2,?3,?4))))";*/
            std::string weights = "update " + dbGridTable + " set weight = distweight from ("
            "select gid as gidsel, min(distance(centroid(" + dbGridTable + ".geometry), i)) as distweight, " + dbGridTable + ".geometry as geom from " + dbGridTable + ",(select geometry as i from " + landTable + " WHERE ROWID IN ("
            "SELECT ROWID FROM SpatialIndex "
            "WHERE f_table_name = '" + landTable + "' AND "
            "search_frame = BuildMbr(" + std::to_string(minX) + "," + std::to_string(minY) + "," + std::to_string(maxX) + "," + std::to_string(maxY) + "))) group by gid) where gid = gidsel";

            // Create sql statements
            sqlite3_stmt* m_createHandle;
            sqlite3_stmt* m_deleteLandHandle;
            sqlite3_stmt* m_weightsHandle;

            // Prepare sql statements (compile)
            sqlite3_prepare_v3(m_db, create.c_str(), -1, SQLITE_PREPARE_PERSISTENT, &m_createHandle, 0);
            //sqlite3_prepare_v3(m_db, deleteLandCells.c_str(), -1, SQLITE_PREPARE_PERSISTENT, &m_deleteLandHandle, 0);
            //sqlite3_prepare_v3(m_db, weights.c_str(), -1, SQLITE_PREPARE_PERSISTENT, &m_weightsHandle, 0);
            //std::cout << "Query - " << deleteLandCells << std::endl;
            // Bind values to sql statements
            {
                sqlite3_bind_double(m_createHandle,1,minX);
                sqlite3_bind_double(m_createHandle,2,minY);
                sqlite3_bind_double(m_createHandle,3,maxX);
                sqlite3_bind_double(m_createHandle,4,maxY);
            }
            /*{
                sqlite3_bind_double(m_deleteLandHandle,1,minX);
                sqlite3_bind_double(m_deleteLandHandle,2,minY);
                sqlite3_bind_double(m_deleteLandHandle,3,maxX);
                sqlite3_bind_double(m_deleteLandHandle,4,maxY);
            }
            {
                sqlite3_bind_double(m_weightsHandle,1,minX);
                sqlite3_bind_double(m_weightsHandle,2,minY);
                sqlite3_bind_double(m_weightsHandle,3,maxX);
                sqlite3_bind_double(m_weightsHandle,4,maxY);
            }*/

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
            int rc = sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);
// polygonFromMultipolygon
            if (sqlite3_prepare_v2(m_db, polygonFromMultipolygon.c_str(), polygonFromMultipolygon.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", polygonFromMultipolygon.c_str());
            }
            // Execute
            rc = sqlite3_step(m_handle);

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
            rc = sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);   
// weightsHandle
/*
            // Execute
            if(sqlite3_step(m_weightsHandle) == SQLITE_ROW) {
                sqlite3_reset(m_weightsHandle);
            } else {
                sqlite3_reset(m_weightsHandle);
            }*/
            if (sqlite3_prepare_v2(m_db, weights.c_str(), weights.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", weights.c_str());
            }
            // Execute
            rc = sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);   

        // Finalize statements
            sqlite3_finalize(m_createHandle);
            //sqlite3_finalize(m_deleteLandHandle);
            //sqlite3_finalize(m_weightsHandle);

        }
        bool SearchGrid::setGridWeights(std::string dbGridTable){

            return false;
        }
        void SearchGrid::deleteGrid(std::string dbGridTable){
            std::string deleteQuery = "select DropGeoTable('" + dbGridTable + "')";
            std::string deleteQueryraw = "select DropGeoTable('" + dbGridTable + "raw')";
            int errors = 0;
            sqlite3_stmt* m_handle;

            if (sqlite3_prepare_v2(m_db, deleteQuery.c_str(), deleteQuery.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", recoverMultiTable.c_str());
            }
         
            // Execute
            int rc = sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);

            if (sqlite3_prepare_v2(m_db, deleteQueryraw.c_str(), deleteQueryraw.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;//Error("Failed to prepare statement", recoverMultiTable.c_str());
            }
         
            // Execute
            rc = sqlite3_step(m_handle);

            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);

        }
        std::vector<int> SearchGrid::calculateSearchPath(double startCell, std::string dbGridTable) {


        }
        std::vector<std::pair<double, double>> SearchGrid::locationsFromCells(std::vector<int> cells, std::string dbGridTable) {
            std::vector<std::pair<double, double>> out;
            for(auto i = cells.begin(); i < cells.end();i++) {
                out.push_back(getCellLocation(*i));
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

        std::pair<double,double> SearchGrid::getCellLocation(int cell, std::string dbGridTable) {
            std::string query = "select X(center), Y(center) from (select centroid(geometry) as center from " + dbGridTable + " where gid = " + std::to_string(cell) + ")";
            int errors = 0;
            sqlite3_stmt* m_handle;

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;
            }
            int m_idx = 0;
            // Execute
            int rc = sqlite3_step(m_handle);
            double X = sqlite3_column_double(m_handle, m_idx++);
            double Y = sqlite3_column_double(m_handle, m_idx++);
            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);

            return std::pair<double,double>(X, Y);
        }
        int SearchGrid::getClosestCell(double X, double Y, std::string dbGridTable) {
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
            int rc = sqlite3_step(m_handle);
            int value = sqlite3_column_int(m_handle, m_idx++);
            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);

            return value;
        }
        int SearchGrid::getClosestUnsearchedCell(int cell, std::string dbGridTable) {
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
            int rc = sqlite3_step(m_handle);
            int value = sqlite3_column_int(m_handle, m_idx++);
            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);

            return value;
        }
        int SearchGrid::getLocalOptimalNeighbour(int cell, std::string dbGridTable) {
            std::string query = "select gid from (select min(weight) as mw,gid from " + dbGridTable + " where weight > 0 and st_touches(geometry, (select geometry from " + dbGridTable + " where gid = " + std::to_string(cell) + ")))";
            int errors = 0;
            sqlite3_stmt* m_handle;

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;
            }
            int m_idx = 0;
            // Execute
            int rc = sqlite3_step(m_handle);
            int value = sqlite3_column_int(m_handle, m_idx++);
            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);

            return value;
        }
        void SearchGrid::setCellWeight(int cell, int weight, std::string dbGridTable) {
            std::string query = "update " + dbGridTable + " set weight = " + std::to_string(weight) + " where gid = " + std::to_string(cell) + "";
            int errors = 0;
            sqlite3_stmt* m_handle;

            if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &m_handle, 0) != SQLITE_OK)
            {
                errors++;
            }
            // Execute
            int rc = sqlite3_step(m_handle);
            // Teardown
            if (m_handle)
                sqlite3_finalize(m_handle);
        }
}