#include "isPointInLayerStatement.hpp"
namespace ENCGIS
{
  isPointInLayerStatement::isPointInLayerStatement(std::string layer, std::string geometry_column, sqlite3 *db, int geometry_epsg, std::string attached_db) {
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
    std::string query = "select 1 from " + attached_db + "." + layer + " WHERE ROWID IN ("
    "SELECT ROWID FROM SpatialIndex "
    "WHERE f_table_name = 'DB=" + attached_db + "." + layer + "' AND "
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
      sqlite3_reset(m_handle);
      return 1;//statusLastResult;
    } else {
      sqlite3_reset(m_handle);
      return 0;
    }
  }
}