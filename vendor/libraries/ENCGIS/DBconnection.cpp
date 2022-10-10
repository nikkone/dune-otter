#include "DBconnection.hpp"

namespace ENCGIS
{
  DBconnection::DBconnection(std::string filename, int flag, int SRIDin): SRID(SRIDin) {
    int rc = sqlite3_open_v2(filename.c_str(), &db,flag,0);
    if( rc ){
      Error("Can't open database: %s\n", sqlite3_errmsg(db));
      sqlite3_close(db);
    }
    loadSpatialite();
  }

  DBconnection::~DBconnection() {
    sqlite3_close(db);
  }

  bool DBconnection::isPointInLayer(double X, double Y, std::string table, bool useSpatialIndex) {
      std::string sql_stmt;
    if(useSpatialIndex) {
        sql_stmt = "select sum(intersects(MakePoint(" + std::to_string(Y) + ", " + std::to_string(X) + ", " + std::to_string(SRID) + " ), geom)) as c from (SELECT geom FROM " + table + " "
      "WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + table + "' AND "
          "search_frame = BuildMbr(" + std::to_string(Y) + ", " + std::to_string(X) + "," + std::to_string(Y) + ", " + std::to_string(X) + ", " + std::to_string(SRID) + " )));"; 
    } else {
        sql_stmt = "select count(*) from " + table + " as d where intersects(MakePoint(" + std::to_string(Y) + ", " + std::to_string(X) + ", " + std::to_string(SRID) + " ), d.geom) limit 1";
    }
    //std::cout << sql_stmt << std::endl;
    //Setup
    sqlite3_stmt* m_handle;
    
    if (sqlite3_prepare_v2(db, sql_stmt.c_str(), sql_stmt.length(), &m_handle, 0) != SQLITE_OK)
    {
      sqlite3_finalize(m_handle);
      Error("Failed to prepare statement", sql_stmt.c_str());
    }
    // Execute
    int m_idx = 0;
    int value = 0;
    if(sqlite3_step(m_handle) == SQLITE_ROW) {
      if (sqlite3_column_type(m_handle, m_idx) != SQLITE_INTEGER) {
        sqlite3_finalize(m_handle);
        return false;//throw Error("column result is not of INTEGER type", "");
      }
      value = sqlite3_column_int(m_handle, m_idx++);
    }
    // Teardown
    if (m_handle)
      sqlite3_finalize(m_handle);
    return value;
  }

  void DBconnection::loadSpatialite() {

    sqlite3_enable_load_extension(db, 1);
    std::string c_stmt = "SELECT load_extension('mod_spatialite');";
    runQuery(c_stmt);
    sqlite3_enable_load_extension(db, 0);
    runQuery("select spatialite_version();");
  }

  int DBconnection::checkTransectLanding(double startX, double startY, double endX, double endY, std::string table, bool useSpatialIndex) {
    std::string sql_stmt;
    if(useSpatialIndex) {
      // WARNING: This needs rtree module in sqlite, can be enabled by adding set(SQLITE3_C_FLAGS "${SQLITE3_C_FLAGS} -DSQLITE_ENABLE_RTREE=1") to vendor/libraries/sqlite3/Library.cmake
      sql_stmt = "select sum(intersects(makeline(makepoint(" + std::to_string(startX) + ", " + std::to_string(startY) + ", " + std::to_string(SRID) + " ), makepoint(" + std::to_string(endX) + ", " + std::to_string(endY) + ", " + std::to_string(SRID) + " )), geom)) as s from (SELECT geom FROM " + table + " "
      "WHERE ROWID IN ("
      "SELECT ROWID FROM SpatialIndex "
      "WHERE f_table_name = '" + table + "' AND "
      "search_frame = BuildMbr(" + std::to_string(startX) + ", " + std::to_string(startY) + ", " + std::to_string(endX) + ", " + std::to_string(endY) + ", " + std::to_string(SRID) + " )))";
    } else {
      sql_stmt = "select count(*) from " + table + " as l where intersects(GeomFromText(\"LineString(" + std::to_string(startX) + " " + std::to_string(startY) + ", " + std::to_string(endX) + " " + std::to_string(endY) + ")\", " + std::to_string(SRID) + " ), l.geom)";
    }
    sqlite3_stmt* m_handle;
    
    if (sqlite3_prepare_v2(db, sql_stmt.c_str(), sql_stmt.length(), &m_handle, 0) != SQLITE_OK)
    {
      sqlite3_finalize(m_handle);
      Error("Failed to prepare statement", sql_stmt.c_str());
    }
    // Execute
    int m_idx = 0;
    int value = 0;
    if(sqlite3_step(m_handle) == SQLITE_ROW) {
      if (sqlite3_column_type(m_handle, m_idx) != SQLITE_INTEGER) {
        sqlite3_finalize(m_handle);
        return -1;//throw Error("column result is not of INTEGER type", "");
      }
      value = sqlite3_column_int(m_handle, m_idx++);
    }
    // Teardown
    if (m_handle)
      sqlite3_finalize(m_handle);
    return value;

  }

  void DBconnection::runQuery(const std::string &sql_stmt) {
      char *zErrMsg = 0;
      if(sqlite3_exec(db, sql_stmt.c_str(), callback, 0, &zErrMsg)!=SQLITE_OK ){
        if(zErrMsg == NULL) {
          Error("SQL error: %s\n", "Received NULL as error message, but SQLITE_OK nt received. Does the databasefile exist?");
        } else {
          Error("SQL error: %s\n", zErrMsg);
        }
        sqlite3_free(zErrMsg);
      }
  }

  int DBconnection::callback(void *NotUsed, int argc, char **argv, char **azColName){
      int i;
      for(i=0; i<argc; i++){
        printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
      }
      printf("\n");
      return 0;
  }

  bool DBconnection::runNoOutputQuery(const std::string &sql_stmt) {
      //Setup
      sqlite3_stmt* m_handle;
      
      if (sqlite3_prepare_v2(db, sql_stmt.c_str(), sql_stmt.length(), &m_handle, 0) != SQLITE_OK)
      {
        Error("Failed to prepare statement", sql_stmt.c_str());
      }
      // Execute
      int rc = sqlite3_step(m_handle);

      // Teardown
      if (m_handle)
        sqlite3_finalize(m_handle);

      if(rc == SQLITE_DONE) {
        return true;
      }
      return false;

  }

  void DBconnection::transformSRID(double in_x, double in_y, unsigned in_srid, double &out_x, double &out_y, unsigned out_srid) {
    std::string sqlstmt = "select X(p), Y(p) from (select transform(makepoint(?1, ?2, ?3), ?4) as p)";
      sqlite3_stmt* m_handle;
      sqlite3_prepare_v3(db, sqlstmt.c_str(), -1, SQLITE_PREPARE_PERSISTENT, &m_handle, 0);
      sqlite3_bind_double(m_handle,1,in_x);
      sqlite3_bind_double(m_handle,2,in_y);
      sqlite3_bind_int(m_handle,3,in_srid);
      sqlite3_bind_int(m_handle,4,out_srid);
        // Execute
      if(sqlite3_step(m_handle) == SQLITE_ROW) {
        out_x = sqlite3_column_int(m_handle, 0);
        out_y = sqlite3_column_int(m_handle, 1);
        sqlite3_reset(m_handle);
      } else {
        sqlite3_reset(m_handle);
      }
      sqlite3_finalize(m_handle);
  }

}
