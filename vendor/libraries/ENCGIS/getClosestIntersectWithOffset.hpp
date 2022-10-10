#ifndef ENCGIS_GETCLOSESTWITHINOFFSET_HPP_INCLUDED
#define ENCGIS_GETCLOSESTWITHINOFFSET_HPP_INCLUDED

// SQLITE3 headers.
#include <sqlite3/sqlite3.h>

// CPP STD headers
#include <string>

namespace ENCGIS
{
  class getClosestIntersectWithOffset {
    public:
    getClosestIntersectWithOffset(std::string layer, std::string geometry_column, sqlite3 *db, int geometry_epsg, double offset = 0.9);
    ~getClosestIntersectWithOffset();
      double run(double startX, double startY, double endX, double endY, double &bestOptionX, double &bestOptionLon);
    private:
      sqlite3_stmt* m_handle;
      double statusLastResult;
  };
}

#endif // ENCGIS_GETCLOSESTWITHINOFFSET_HPP_INCLUDED