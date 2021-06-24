#include "DBconnection.hpp"

namespace ENCGIS {

      isPointInLayerStatement::isPointInLayerStatement(std::string layer, std::string geometry_column, sqlite3 *db, int geometry_epsg) {
        /*std::string query = "select sum(intersects(MakePoint(?1,?2, " + epsg + " ), geom)) as c from (SELECT geom FROM " + layer + " "
      "WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + layer + "' AND "
          "search_frame = BuildMbr(?1,?2,?1,?2, " + epsg + " )));"; */

      //OLD: Does not stop after one intersection is returned
      /*std::string query = "select sum(intersects(MakePoint(?1,?2, " + std::to_string(geometry_epsg) + " ), " + geometry_column + ")) FROM " + layer + " "
      "WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + layer + "' AND "
          "search_frame = BuildMbr(?1,?2,?1,?2));";*/
          
      // New: Stops at first intersection
      std::string query = "select 1 from " + layer + " WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + layer + "' AND "
          "search_frame = BuildMbr(?1,?2,?1,?2)) and "
		    "intersects(MakePoint(?1,?2, " + std::to_string(geometry_epsg) + " ), " + geometry_column + ") limit 1";


        sqlite3_prepare_v3(db, query.c_str(), -1, SQLITE_PREPARE_PERSISTENT, &m_handle, 0);
      }

      isPointInLayerStatement::~isPointInLayerStatement() {
        if (m_handle)
          sqlite3_finalize(m_handle);
      }

      int isPointInLayerStatement::run(double X, double Y) {
        sqlite3_bind_double(m_handle,1,X);
        sqlite3_bind_double(m_handle,2,Y);
        // Execute
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          statusLastResult = sqlite3_column_int(m_handle, 0);
          sqlite3_reset(m_handle);
          //return sqlite3_stmt_status(m_handle, SQLITE_STMTSTATUS_FULLSCAN_STEP, false);
          return statusLastResult;
        } else {
          sqlite3_reset(m_handle);
          return 0;
        }
      }

      lineIntersectLayerStatement::lineIntersectLayerStatement(std::string layer, std::string geometry_column, sqlite3 *db, int geometry_epsg) {
        //OLD: Does not stop after one polygon is returned
     /*std::string query = "select sum(intersects(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + " ), makepoint(?3,?4, " + std::to_string(geometry_epsg) + ")), " + geometry_column + ")) FROM " + layer + " "
      "WHERE ROWID IN ("
      "SELECT ROWID FROM SpatialIndex "
      "WHERE f_table_name = '" + layer + "' AND "
      "search_frame = BuildMbr(?1,?2,?3,?4))";*/
        // New: Stops at first intersection
    std::string query = "select 1 FROM '" + layer + "' "
	  "WHERE ROWID IN ("
		"SELECT ROWID FROM SpatialIndex "
		"WHERE f_table_name = '" + layer + "' AND "
		"search_frame = BuildMbr(?1,?2,?3,?4)) AND "
		"intersects(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + " ), makepoint(?3,?4, " + std::to_string(geometry_epsg) + ")), " + geometry_column + ") limit 1";

      //printf("\n%s\n", query.c_str());
        sqlite3_prepare_v3(db, query.c_str(), -1, SQLITE_PREPARE_PERSISTENT, &m_handle, 0);
      }

      lineIntersectLayerStatement::~lineIntersectLayerStatement() {
        if (m_handle)
          sqlite3_finalize(m_handle);
      }

      int lineIntersectLayerStatement::run(double startX, double startY, double endX, double endY) {
        //printf("%f, %f, %f, %f\n", startX,  startY,  endX,  endY);
        sqlite3_bind_double(m_handle,1,startX);
        sqlite3_bind_double(m_handle,2,startY);
        sqlite3_bind_double(m_handle,3,endX);
        sqlite3_bind_double(m_handle,4,endY);
        // Execute
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          statusLastResult = sqlite3_column_int(m_handle, 0);
          sqlite3_reset(m_handle);
          //return sqlite3_stmt_status(m_handle, SQLITE_STMTSTATUS_FULLSCAN_STEP, false);
        //  printf("\nOne\n");
          return statusLastResult;
        } else {
          sqlite3_reset(m_handle);
        //  printf("\nZero\n");
          return 0;
        }
      }
/////////////////////////////////////////////////////////////////////////////
      getClosestIntersectWithOffset::getClosestIntersectWithOffset(std::string layer, std::string geometry_column, sqlite3 *db, int geometry_epsg, double offset) {
    /*std::string query = 
    "select Line_Locate_Point(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + " ), makepoint(?3,?4, " + std::to_string(geometry_epsg) + ")), p), X(p), Y(p) from ("
    "select s,Line_Interpolate_Point(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + " ),s), 0.9) as p, distance(s, makepoint(?1,?2, " + std::to_string(geometry_epsg) + " )) as d from ("
    "select StartPoint(Intersection(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + " ), makepoint(?3,?4, " + std::to_string(geometry_epsg) + ")), " + geometry_column + ")) as s from " + layer + " WHERE ROWID IN ("
    "    SELECT ROWID FROM SpatialIndex "
    "    WHERE f_table_name = '" + layer + "' AND "
    "      search_frame = BuildMbr(?1,?2,?3,?4)) and "
		"    intersects(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + " ), makepoint(?3,?4, " + std::to_string(geometry_epsg) + ")), " + geometry_column + ")"
    ") order by d asc limit 1)";*/
    std::string query =
    "select Line_Locate_Point(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + "), makepoint(?3,?4, " + std::to_string(geometry_epsg) + ")), p), X(p), Y(p) from ("
    "select Line_Interpolate_Point(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + "),s), "+std::to_string(offset)+") as p from ("
    "select StartPoint(Intersection(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + "), makepoint(?3,?4, " + std::to_string(geometry_epsg) + ")), " + geometry_column + ")) as s from ("
    "select " + geometry_column + " from ("
    "select * from " + layer + " WHERE ROWID IN ("
    "SELECT ROWID FROM SpatialIndex "
    "WHERE f_table_name = '" + layer + "' AND "
    "search_frame = BuildMbr(?1,?2,?3,?4))"
		") where intersects(makeline(makepoint(?1,?2, " + std::to_string(geometry_epsg) + "), makepoint(?3,?4, " + std::to_string(geometry_epsg) + ")), " + geometry_column + ")"
		") order by distance(s, makepoint(?1,?2, " + std::to_string(geometry_epsg) + ")) asc limit 1"
		")"
		")";
    
      //printf("\n%s\n", query.c_str());
        sqlite3_prepare_v3(db, query.c_str(), -1, SQLITE_PREPARE_PERSISTENT, &m_handle, 0);
      }

      getClosestIntersectWithOffset::~getClosestIntersectWithOffset() {
        if (m_handle)
          sqlite3_finalize(m_handle);
      }

      double getClosestIntersectWithOffset::run(double startX, double startY, double endX, double endY, double &bestOptionX, double &bestOptionY) {
        //printf("li2: %f, %f, %f, %f\n", startX,  startY,  endX,  endY);
        sqlite3_bind_double(m_handle,1,startX);
        sqlite3_bind_double(m_handle,2,startY);
        sqlite3_bind_double(m_handle,3,endX);
        sqlite3_bind_double(m_handle,4,endY);

        // Execute
        if(sqlite3_step(m_handle) == SQLITE_ROW) {
          statusLastResult = sqlite3_column_double(m_handle, 0);
          bestOptionX = sqlite3_column_double(m_handle, 1);
          bestOptionY = sqlite3_column_double(m_handle, 2);
          sqlite3_reset(m_handle);
          //return sqlite3_stmt_status(m_handle, SQLITE_STMTSTATUS_FULLSCAN_STEP, false);
        //  printf("\nOne\n");
          return statusLastResult;
        } else {
          sqlite3_reset(m_handle);
        //  printf("\nZero\n");
          return 1.0;
        }
      }
/////////////////////////////////////////////////////////////////////////////

  bool DBconnection::isPointInLayer(double X, double Y, std::string table, bool useSpatialIndex) {
      std::string sql_stmt;
    if(useSpatialIndex) {
        sql_stmt = "select sum(intersects(MakePoint(" + std::to_string(Y) + ", " + std::to_string(X) + ", " + std::to_string(epsg) + " ), geom)) as c from (SELECT geom FROM " + table + " "
      "WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + table + "' AND "
          "search_frame = BuildMbr(" + std::to_string(Y) + ", " + std::to_string(X) + "," + std::to_string(Y) + ", " + std::to_string(X) + ", " + std::to_string(epsg) + " )));"; 
    } else {
        sql_stmt = "select count(*) from " + table + " as d where intersects(MakePoint(" + std::to_string(Y) + ", " + std::to_string(X) + ", " + std::to_string(epsg) + " ), d.geom) limit 1";
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
    //std::string ext=""
    /*char *zErrMsg = 0;
    if(sqlite3_load_extension(db, "mod_spatialite", 0, &zErrMsg) != SQLITE_OK ) {
      printf("notok");
      sqlite3_free(zErrMsg);
    }*/

    sqlite3_enable_load_extension(db, 1);
    std::string c_stmt = "SELECT load_extension('mod_spatialite');";
    runQuery(c_stmt);
    sqlite3_enable_load_extension(db, 0);
    runQuery("select spatialite_version();");
  }

    DBconnection::DBconnection(std::string filename, int flag, int epsgIn): epsg(epsgIn) {

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

  int DBconnection::checkTransectLanding(double startX, double startY, double endX, double endY, std::string table, bool useSpatialIndex) {
    std::string sql_stmt;
    if(useSpatialIndex) {
      // WARNING: This needs rtree module in sqlite, can be enabled by adding set(SQLITE3_C_FLAGS "${SQLITE3_C_FLAGS} -DSQLITE_ENABLE_RTREE=1") to vendor/libraries/sqlite3/Library.cmake
      sql_stmt = "select sum(intersects(makeline(makepoint(" + std::to_string(startX) + ", " + std::to_string(startY) + ", " + std::to_string(epsg) + " ), makepoint(" + std::to_string(endX) + ", " + std::to_string(endY) + ", " + std::to_string(epsg) + " )), geom)) as s from (SELECT geom FROM " + table + " "
      "WHERE ROWID IN ("
      "SELECT ROWID FROM SpatialIndex "
      "WHERE f_table_name = '" + table + "' AND "
      "search_frame = BuildMbr(" + std::to_string(startX) + ", " + std::to_string(startY) + ", " + std::to_string(endX) + ", " + std::to_string(endY) + ", " + std::to_string(epsg) + " )))";
    } else {
      sql_stmt = "select count(*) from " + table + " as l where intersects(GeomFromText(\"LineString(" + std::to_string(startX) + " " + std::to_string(startY) + ", " + std::to_string(endX) + " " + std::to_string(endY) + ")\", " + std::to_string(epsg) + " ), l.geom)";
    }
    /*
    std::pair<bool, int> DBDepth;
    iterator_stmt->execute();
    *iterator_stmt >> DBDepth;
    if(std::get<0>(DBDepth)) {
      delete iterator_stmt;
      return std::get<1>(DBDepth);
    } else {
      delete iterator_stmt;
      return -1;
    }*/
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

  void DBconnection::runQuery(std::string sqlstmt) {
      char *zErrMsg = 0;
      if(sqlite3_exec(db, sqlstmt.c_str(), callback, 0, &zErrMsg)!=SQLITE_OK ){
        Error("SQL error: %s\n", zErrMsg);
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

  bool DBconnection::runNoOutputQuery(std::string sql_stmt) {
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
}
