#ifndef ENCGIS_LINEINTERSECTLAYERSTATEMENT_HPP_INCLUDED
#define ENCGIS_LINEINTERSECTLAYERSTATEMENT_HPP_INCLUDED

// SQLITE3 headers.
#include <sqlite3/sqlite3.h>

// CPP STD headers
#include <string>

namespace ENCGIS
{
  class lineIntersectLayerStatement {
    public:
    lineIntersectLayerStatement(std::string layer, std::string geometry_column, sqlite3 *db, int geometry_epsg, std::string attached_db = "main");
    ~lineIntersectLayerStatement();
      int run(double startLat, double startLon, double endLat, double endLon);
    private:
      sqlite3_stmt* m_handle;
      //int statusLastResult;
  };
}

#endif // ENCGIS_LINEINTERSECTLAYERSTATEMENT_HPP_INCLUDED