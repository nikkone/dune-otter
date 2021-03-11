//***************************************************************************
// Copyright 2020-2021 Norwegian University of Science and Technology       *
// Department of Engineering Technology                                     *
//***************************************************************************
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

#ifndef ENCGIS_HPP_INCLUDED
#define ENCGIS_HPP_INCLUDED

// ISO C++ 98 headers.
#include <string>
#include <stdexcept>
#include <vector>
#include <random>

// DUNE headers.
#include <DUNE/Config.hpp>
#include <DUNE/Database/Connection.hpp>
#include <DUNE/IMC.hpp>
#include <DUNE/System/Error.hpp>

namespace ENCGIS
{
  //! Database connection with functions for using Spatialite stored ENCs
  class ChartsDBConnection : public DUNE::Database::Connection
  {
  public:
    class Error: public std::runtime_error
    {
    public:
      Error(std::string op, std::string msg):
        std::runtime_error("Error (" + op + "): " + msg)
      { }
    };
      //! Opens/Constructs a file-based database and loads the Spatialite extension.
      //! @param path database file
      //! @param flags connection flags (@see Database::Connection::ConnectionFlags)
      ChartsDBConnection(const char* path, int flags);

      //! Runs a query in the DB that finds the lowest expected depth occuring in
      //! a straight line between start and end. This is designed to be used
      //! on a table with data from S-57 DEPARE containing 'geom' in WGS84 degrees and 'DRVAL1' in meters.
      //! @param[in] startLat The latitude of the starting point given in degrees.
      //! @param[in] startLon The longtitude of the starting point given in degrees.
      //! @param[in] endLat The latitude of the ending point given in degrees.
      //! @param[in] endLon The longtitude of the ending point given in degrees.
      //! @param[in] table The table in the database containing DEPARE data.
      //! @return The minimum depth encountered on the transect given in meters below surface, -1 if the query returns NULL.
      double checkTransectMinDepth(double startLat, double startLon, double endLat, double endLon, std::string table="deparetable");

      //! Runs a query in the DB that counts how many times a straight line between
      //! start and end crosses areas of land. This is designed to be used
      //! on a table with data from S-57 LNDARE containing 'geom' in WGS84 degrees.
      //! @param[in] startLat The latitude of the starting point given in degrees.
      //! @param[in] startLon The longtitude of the starting point given in degrees.
      //! @param[in] endLat The latitude of the ending point given in degrees.
      //! @param[in] endLon The longtitude of the ending point given in degrees.
      //! @param[in] table The table in the database containing LNDARE data. Default = "lndaretable"
      //! @param[in] useSpatialIndex If spatial index should be used before intersection. Default= true
      //! @return Number of land crossings, -1 if the query returns NULL .
      int checkTransectLanding(double startLat, double startLon, double endLat, double endLon, std::string table="lndaretable", bool useSpatialIndex=true);

      //! Finds a random point in a layer, limited in extent.
      //! @param[in] minX The minimum longtitude in the extent, given in degrees.
      //! @param[in] minY The minimum latitude in the extent, given in degrees.
      //! @param[in] maxX The maximum longtitude in the extent, given in degrees.
      //! @param[in] maxY The maximum latitude in the extent, given in degrees.
      //! @param[in] table The table in the database defining a valid layer.
      //! @return The random point as a longtitude, latitude pair in degrees.
      std::pair<double, double> getRandomValidPointWithinExtent(double minX,double minY,double maxX,double maxY, std::string table="deparetable");

      //! Checks whether a point lies within the the polygons of a layer.
      //! @param[in] lat The minimum longtitude in the extent, given in degrees.
      //! @param[in] lon The minimum latitude in the extent, given in degrees.
      //! @param[in] table The table in the database defining the layer.
      //! @param[in] useSpatialIndex If spatial index should be used before intersection. Default= true
      //! @return True if given point is in layer and false if not.
      bool isPointInLayer(double lat, double lon, std::string table="deparetable", bool useSpatialIndex=true);

      //! Runs a query in the DB that finds the lowest expected depth occuring in
      //! a plan containing only GoTo maneuvers. The current position can be given 
      //! to also check the transect between the current position and the first GoTo position.
      //! The query is designed to be used on a table with data from S-57 DEPARE
      //! containing 'geom' in WGS84 degrees and 'DRVAL1' in meters.
      //! TODO: Implement checking of more maneuvers.
      //! TODO: Check if WKB is faster than WKT
      //! @param[in] plan The checked plan. Should only contain GoTo maneuvers.
      //! @param[in] currentLat The current latitude. If -1.0, then the check is not performed. Given in degrees.
      //! @param[in] curentLon The current longtitude. If -1.0, then the check is not performed. Given in degrees.
      //! @param[in] table The table in the database containing DEPARE data.
      //! @return The minimum depth encountered along the planned path given in meters below surface, -1 if the query returns NULL.
      double checkPlanMinDepth(const DUNE::IMC::PlanSpecification* plan, double currentLat=-1.0, double curentLon=-1.0, std::string table="deparetable");

      //! Runs a query in the DB that counts how many times a plan containing only GoTo maneuvers 
      //! crosses areas of land. The current position can be given 
      //! to also check the transect between the current position and the first GoTo position.
      //! The query is designed to be used on a table with data from S-57 LNDARE
      //! containing 'geom' in WGS84 degrees.
      //! TODO: Implement checking of more maneuvers.
      //! TODO: Check if WKB is faster than WKT
      //! @param[in] plan The checked plan. Should only contain GoTo maneuvers.
      //! @param[in] currentLat The current latitude. If -1.0, then the check is not performed. Given in degrees.
      //! @param[in] curentLon The current longtitude. If -1.0, then the check is not performed. Given in degrees.
      //! @param[in] table The table in the database containing DEPARE data.
      //! @return True if plan contains at least one landing, false if not.
      bool checkPlanLanding(const DUNE::IMC::PlanSpecification* plan, double currentLat=-1.0, double curentLon=-1.0, std::string table="lndaretable");

      //! Converts a plan to an equivalent geometry in well-known text format. Currently only supports GoTo Meneuvers.
      //! TODO: Implement checking of more maneuvers.
      //! @param[in] plan The checked plan. Should only contain GoTo maneuvers.
      //! @param[in] currentLat The current latitude. If -1.0, then the check is not performed. Given in degrees.
      //! @param[in] curentLon The current longtitude. If -1.0, then the check is not performed. Given in degrees.
      //! @return The plan geometry in well-known text format
      std::string planToWKT(const DUNE::IMC::PlanSpecification* plan, double currentLat=-1.0, double curentLon=-1.0);

      //! Reports the extent of a layer in the database.
      //! @param[in] table The table in the database containing DEPARE data.
      //! @param[out] extent_min_x The minimum longtitude in the extent, given in degrees.
      //! @param[out] extent_min_y The minimum latitude in the extent, given in degrees.
      //! @param[out] extent_max_x The maximum longtitude in the extent, given in degrees.
      //! @param[out] extent_max_y The maximum latitude in the extent, given in degrees.
      //! @return True if the query does not return NULL, false if
      bool getExtent(std::string table, double &extent_min_x, double &extent_min_y, double &extent_max_x, double &extent_max_y);

      bool findFirstIntersectingPoint(double startLat, double startLon, double endLat, double endLon, double &pointLat, double &pointLon, double &dbtime, std::string table="lndaretable");
  private:
      std::mt19937 eng;
  };
}

#endif //ENCGIS_HPP_INCLUDED
