#include "lineIntersectLayerStatement.hpp"
namespace ENCGIS
{

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
      sqlite3_reset(m_handle);
      return 1; // As long as the query returns a row, we know that there is at least one intersection.
    } else {
      sqlite3_reset(m_handle);
      return 0;
    }
  }
}