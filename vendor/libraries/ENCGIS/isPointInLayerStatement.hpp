#ifndef ENCGIS_ISPOINTINLAYERSTATEMENT_HPP_INCLUDED
#define ENCGIS_ISPOINTINLAYERSTATEMENT_HPP_INCLUDED

// SQLITE3 headers.
#include <sqlite3/sqlite3.h>

// CPP STD headers
#include <string>

namespace ENCGIS
{
  class isPointInLayerStatement {
    public:
    isPointInLayerStatement(std::string layer, std::string geometry_column, sqlite3 *db, int geometry_epsg);
    ~isPointInLayerStatement();
      int run(double lat, double lon);
    private:
      sqlite3_stmt* m_handle;
      //int statusLastResult;
  };
}
#endif // ENCGIS_ISPOINTINLAYERSTATEMENT_HPP_INCLUDED