#include "getClosestIntersectWithOffset.hpp"
namespace ENCGIS
{
  getClosestIntersectWithOffset::getClosestIntersectWithOffset(std::string layer, std::string geometry_column, sqlite3 *db, int geometry_epsg, double offset) {
    m_handle = nullptr;
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
}