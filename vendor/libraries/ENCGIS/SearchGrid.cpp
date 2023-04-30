#include "SearchGrid.hpp"

#include <iostream>
namespace ENCGIS {
    SearchGrid::SearchGrid(ENCGIS::DBconnection *in_db, std::string in_dbGridTable, unsigned in_SRID, std::string in_obstacleTable) {
        m_con = in_db;
        obstacleTable = in_obstacleTable;
        dbGridTable = in_dbGridTable;
        SRID = in_SRID;
        obstacleTabledb = "db1";
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

    bool SearchGrid::createGrid(double minX, double minY, double maxX, double maxY, unsigned gridsize, gridtypes_t gridType, bool spatialIndex){
        std::string EWKTsquare = "SRID=" + std::to_string(SRID) + ";POLYGON((" + std::to_string(minX) + " " + std::to_string(minY) + "," + std::to_string(minX) + " " + std::to_string(maxY) + ","
        "" + std::to_string(maxX) + " " + std::to_string(maxY) + "," + std::to_string(maxX) + " " + std::to_string(minY) + "," + std::to_string(minX) + " " + std::to_string(minY) + "))";

        return createGrid(EWKTsquare, gridsize, gridType, spatialIndex);
    }






    bool SearchGrid::createGrid(const std::string &EWKTpolygon, unsigned gridsize, gridtypes_t gridType, bool spatialIndex) {

        std::string create = "create table " + dbGridTable + "raw as select " + gridTypeToString(gridType) + "Grid(transform("
        "GeomFromEWKT('" + EWKTpolygon + "'), " + std::to_string(SRID) + "), " +std::to_string(gridsize)+ ") as geometry";
        
        std::string addEffort= "ALTER TABLE " + dbGridTable + "raw ADD COLUMN effort REAL";
        std::string addWeight= "ALTER TABLE " + dbGridTable + "raw ADD COLUMN weight REAL";
        std::string initMetrics = "update " + dbGridTable + "raw SET effort = 0.0, weight = 0.0";
        std::string recoverMultiTable = "SELECT RecoverGeometryColumn('" + dbGridTable + "raw', 'geometry', " + std::to_string(SRID) + ", 'MULTIPOLYGON', 'XY')";

        std::string polygonFromMultipolygon = "SELECT ElementaryGeometries('" + dbGridTable + "raw', 'geometry', '" + dbGridTable + "temp','gid','delme') as geom FROM " + dbGridTable + "raw";

        std::string removeDelme= "ALTER TABLE " + dbGridTable + "temp DROP COLUMN delme";

        std::string deleteLandCells = "delete from " + dbGridTable + "temp where gid in (select " + dbGridTable + "temp.gid from " + dbGridTable + "temp, (select geometry from " + obstacleTabledb + "." + obstacleTable + " where " + obstacleTabledb + "." + obstacleTable + ".ROWID IN ("
    "SELECT ROWID FROM SpatialIndex "
    "WHERE f_table_name = 'DB=" + obstacleTabledb + "." + obstacleTable + "' AND "
        "search_frame = (select GetLayerExtent('" + dbGridTable + "temp')))) as land where intersects(" + dbGridTable + "temp.geometry, land.geometry))";


        std::string createCentroidTable = 
        "CREATE TABLE " + dbGridTable + " ("
            "\"gid\"	INTEGER,"
            "\"effort\"	REAL,"
            "\"weight\"	REAL,"
            "\"geometry\"	POLYGON, "
            "\"center\"	POINT,"
            "PRIMARY KEY(\"gid\" AUTOINCREMENT)"
        ")";
        std::string populateCentroidTable = "insert into " + dbGridTable + " select *, centroid(geometry) as center from " + dbGridTable + "temp;";
        std::string recoverCentroidTable = "SELECT RecoverGeometryColumn('" + dbGridTable + "', 'center', " + std::to_string(SRID) + ", 'POINT', 'XY')";
        std::string recoverGeometryTable = "SELECT RecoverGeometryColumn('" + dbGridTable + "', 'geometry', " + std::to_string(SRID) + ", 'POLYGON', 'XY')";
        //std::string dropTempTable = ;
        std::string createCentroidTableIndex = "select createSpatialIndex('" + dbGridTable + "','center')";


        //std::cout << polygonFromMultipolygon << std::endl;
        std::string indexQuery = "SELECT CreateSpatialIndex('" + dbGridTable + "', 'geometry');";

        m_con->runNoOutputQuery(create);
        m_con->runNoOutputQuery(addEffort);
        m_con->runNoOutputQuery(addWeight);
        m_con->runNoOutputQuery(initMetrics);
        m_con->runNoOutputQuery(recoverMultiTable);
        m_con->runNoOutputQuery(polygonFromMultipolygon);
        m_con->runNoOutputQuery(removeDelme);
        m_con->runNoOutputQuery(deleteLandCells);

        m_con->runNoOutputQuery(createCentroidTable);
        m_con->runNoOutputQuery(populateCentroidTable);
        m_con->runNoOutputQuery(recoverCentroidTable);
        bool ret = m_con->runNoOutputQuery(recoverGeometryTable);
        if(spatialIndex) {
            m_con->runNoOutputQuery(createCentroidTableIndex);
            ret = m_con->runNoOutputQuery(indexQuery);
        }
        return ret;
    }

    bool SearchGrid::setGridMetricFromLandDistance(std::string metric) {
        /*std::string weights = "update " + dbGridTable + " set weight = distweight from ("
        "select gid as gidsel, min(distance(centroid(" + dbGridTable + ".geometry), i)) as distweight, " + dbGridTable + ".geometry as geom from " + dbGridTable + ",(select geometry as i from " + obstacleTabledb + "." + obstacleTable + " WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = 'DB=" + obstacleTabledb + "." + obstacleTable + "' AND "
        "search_frame = (select GetLayerExtent('" + dbGridTable + "')))) group by gid) where gid = gidsel";*/
        std::string weights = "update " + dbGridTable + " set " + metric + " = distweight from ("
        "select g,gid as gidsel, min(distance(centroid(" + dbGridTable + ".geometry), i)) as distweight, " + dbGridTable + ".geometry as geom from " + dbGridTable + ",(select t.'group' as g, geometry as i from " + obstacleTabledb + "." + obstacleTable + " as t WHERE"
        " ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = 'DB=" + obstacleTabledb + "." + obstacleTable + "' AND "
        "search_frame = (select GetLayerExtent('" + dbGridTable + "', 'geometry')))) group by gid,g ) where gid = gidsel and g = 13";
        //std::cout << weights << std::endl;
        return m_con->runNoOutputQuery(weights);
    }

    void SearchGrid::deleteGrid() {
        std::string deleteQuery = "select DropTable(NULL, '" + dbGridTable + "', TRUE)";
        std::string deleteQueryraw = "select DropTable(NULL, '" + dbGridTable + "raw', TRUE)";
        std::string deleteQuerytemp = "select DropTable(NULL, '" + dbGridTable + "temp', TRUE)";
        m_con->runNoOutputQuery(deleteQuery);
        m_con->runNoOutputQuery(deleteQueryraw);
        m_con->runNoOutputQuery(deleteQuerytemp);

    }

    std::vector<std::pair<double, double>> SearchGrid::locationsFromCells(const std::vector<int> &cells, unsigned outputSRID) {
        std::vector<std::pair<double, double>> out;
        for(auto i = cells.begin(); i < cells.end();i++) {
            out.push_back(getCellLocation(*i, outputSRID));
        }
        return out;
    }

    bool SearchGrid::makeMetricSumToOne(std::string metric) {
        std::string sumToOneScale = "update " + dbGridTable + " set " + metric + "=" + metric + "/(select sum(" + metric + ") from " + dbGridTable + ")";
        return m_con->runNoOutputQuery(sumToOneScale);
    }

    void SearchGrid::normalizeMetric(bool invert, std::string metric) {
      // Find max/min weight
      std::string maxmin = "select min(" + metric + "), max(" + metric + ") from " + dbGridTable;
      sqlite3_stmt* m_handle = nullptr;

      if (sqlite3_prepare_v2(m_con->db, maxmin.c_str(), maxmin.length(), &m_handle, 0) == SQLITE_OK) {
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          double min = sqlite3_column_double(m_handle, 0);
          double max = sqlite3_column_double(m_handle, 1);
          sqlite3_finalize(m_handle);
          // Recalculate weights
          std::string recalculateWeights;
          if(invert) {
              recalculateWeights = "update " + dbGridTable + " set " + metric + " = max(0.0, 1.0 + (" + std::to_string(min) + " - " + metric + ")/" + std::to_string(max-min) + ")";
          } else {
              recalculateWeights = "update " + dbGridTable + " set " + metric + " = max(0.0, (" + metric + " - " + std::to_string(min) + ")/" + std::to_string(max-min) + ")";
          }

          if (sqlite3_prepare_v2(m_con->db, recalculateWeights.c_str(), recalculateWeights.length(), &m_handle, 0) == SQLITE_OK) {
            sqlite3_step(m_handle);
          }
        }
      }
      sqlite3_finalize(m_handle);
    }

    std::pair<double,double> SearchGrid::getCellLocation(int cell, unsigned outputSRID) {
      //std::string query = "select X(center), Y(center) from (select transform(centroid(geometry), " + std::to_string(outputSRID) + ") as center from " + dbGridTable + " where gid = " + std::to_string(cell) + ")";
      std::string query = "select X(center), Y(center) from (select transform(center, " + std::to_string(outputSRID) + ") as center from " + dbGridTable + " where gid = " + std::to_string(cell) + ")";

      sqlite3_stmt* m_handle = nullptr;
      double X = 0.0;
      double Y = 0.0;
      if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          X = sqlite3_column_double(m_handle, 0);
          Y = sqlite3_column_double(m_handle, 1);
        }
      }
      sqlite3_finalize(m_handle);
      return std::pair<double,double>(X, Y);
    }
    
    int SearchGrid::getClosestCell(double X, double Y) {
      std::string query = "select gid from ("
                          "select gid, min(distance(geometry, makepoint(" + std::to_string(X) + "," + std::to_string(Y) + ", 32632))) from " + dbGridTable + ")";
      int value = 0;
      sqlite3_stmt* m_handle = nullptr;

      if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          value = sqlite3_column_int(m_handle, 0);
        }
      }

      sqlite3_finalize(m_handle);
      return value;
    }
    
    int SearchGrid::getClosestUnsearchedCell(int cell, std::string metric) {
        std::string query = "select gid from ("
                            "select gid, min(distance(geometry, (select geometry from " + dbGridTable + " where gid = " + std::to_string(cell) + "))) from " + dbGridTable + " where " + metric + " > 0)";
        int value = 0;
        sqlite3_stmt* m_handle = nullptr;

      if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          value = sqlite3_column_int(m_handle, 0);
        }
      }
      sqlite3_finalize(m_handle);
      return value;
    }

    double SearchGrid::getAzimuth(int cell1, int cell2) {
      std::string query = " select azimuth((select center from " + dbGridTable + " where gid = " + std::to_string(cell1) + "), (select center from " + dbGridTable + " where gid = " + std::to_string(cell2) + "))";
      double azimuth = 0.0;
      sqlite3_stmt* m_handle = nullptr;

      if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &m_handle, 0) == SQLITE_OK) {
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
        azimuth = sqlite3_column_double(m_handle, 0);
        }
      }
      sqlite3_finalize(m_handle);
      return azimuth;
    }

    bool SearchGrid::setCellMetric(int cell, int value, std::string metric) {
        std::string query = "update " + dbGridTable + " set " + metric + " = " + std::to_string(value) + " where gid = " + std::to_string(cell) + "";
        return m_con->runNoOutputQuery(query);
    }
}