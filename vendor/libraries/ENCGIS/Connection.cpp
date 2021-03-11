//***************************************************************************
// Copyright 2020-2021 Norwegian University of Science and Technology       *
// Department of Engineering Technology                                     *
//***************************************************************************
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************


// ISO C++ 98 headers.
#include <iostream>
#include <iomanip>      // std::setprecision

// SQLITE3 headers.
#include <sqlite3/sqlite3.h>

// DUNE headers.
#include "Connection.hpp"
#include <DUNE/Coordinates.hpp>
#include <DUNE/Database/Statement.hpp>


/* TODO
Add topology following.
Remove WKT points, use makePoint
Fix checkTransectMinDepth
// Create Table, possibly temp


*/
namespace ENCGIS
{
  ChartsDBConnection::ChartsDBConnection(const char* path, int flags):Connection(path, flags)  {
    //Enables extensions in SQLite3
    sqlite3_enable_load_extension(handle(), 1);
    // Load SpatiaLite extension
    std::string c_stmt = "SELECT load_extension('mod_spatialite.so');";
    DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this);
    iterator_stmt->execute();
    delete iterator_stmt;
    // Set seed for randomgenerators
    std::random_device r;
    std::seed_seq seed{r(), r(), r(), r(), r(), r(), r(), r()};
    eng = std::mt19937(seed);
  }

  double ChartsDBConnection::checkTransectMinDepth(double startLat, double startLon, double endLat, double endLon, std::string table) {
    std::string c_stmt = "select min(DRVAL1) from " + table + " as d where intersects(GeomFromText(\"LineString(" + std::to_string(startLon) + " " + std::to_string(startLat) + ", " + std::to_string(endLon) + " " + std::to_string(endLat) + ")\", 4326), d.geom);";
    DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this); 
    double minDepth =-1;
    std::pair<bool, double> DBDepth;

    while(iterator_stmt->execute())
    {
      *iterator_stmt >> DBDepth;
      
      if(std::get<0>(DBDepth)) // Check if answer not NULL
      {
        minDepth = std::get<1>(DBDepth);
      }
    }
        iterator_stmt->reset();
        delete iterator_stmt;
    return minDepth;
  }

  int ChartsDBConnection::checkTransectLanding(double startLat, double startLon, double endLat, double endLon, std::string table, bool useSpatialIndex) {
    DUNE::Database::Statement* iterator_stmt;

    if(useSpatialIndex) {
      // WARNING: This needs rtree module in sqlite, can be enabled by adding set(SQLITE3_C_FLAGS "${SQLITE3_C_FLAGS} -DSQLITE_ENABLE_RTREE=1") to vendor/libraries/sqlite3/Library.cmake
      std::string c_stmt = "select sum(intersects(makeline(makepoint(" + std::to_string(startLon) + ", " + std::to_string(startLat) + ", 4326), makepoint(" + std::to_string(endLon) + ", " + std::to_string(endLat) + ", 4326)), geom)) as s from (SELECT geom FROM " + table + " "
      "WHERE ROWID IN ("
      "SELECT ROWID FROM SpatialIndex "
      "WHERE f_table_name = '" + table + "' AND "
      "search_frame = BuildMbr(" + std::to_string(startLon) + ", " + std::to_string(startLat) + ", " + std::to_string(endLon) + ", " + std::to_string(endLat) + ", 4326)))";
      iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this); 
      //std::cout << c_stmt << std::endl;
    } else {
      std::string c_stmt = "select count(*) from " + table + " as l where intersects(GeomFromText(\"LineString(" + std::to_string(startLon) + " " + std::to_string(startLat) + ", " + std::to_string(endLon) + " " + std::to_string(endLat) + ")\", 4326), l.geom)";
      iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this); 
      //std::cout << c_stmt << std::endl;
    }

    std::pair<bool, int> DBDepth;
    iterator_stmt->execute();
    *iterator_stmt >> DBDepth;
    if(std::get<0>(DBDepth)) {
      delete iterator_stmt;
      return std::get<1>(DBDepth);
    } else {
      delete iterator_stmt;
      return -1;
    }
/*
    int landings=-1;
    while(iterator_stmt->execute())
    {
      *iterator_stmt >> DBDepth;
      
      if(std::get<0>(DBDepth))
      {
        landings = std::get<1>(DBDepth);
      }
    }
    delete iterator_stmt;
    return landings;*/
  }


  bool ChartsDBConnection::getExtent(std::string table, double &extent_min_x, double &extent_min_y, double &extent_max_x, double &extent_max_y) {
    //std::string c_stmt = "select GetLayerExtent(" + table + ") from " + table + " as d limit 1";
    std::string c_stmt = "SELECT extent_min_x ,extent_min_y,  extent_max_x,  extent_max_y FROM vector_layers_statistics WHERE table_name='" + table + "' AND geometry_column='geom'";
    DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this); 
    std::pair<bool, double> minX_d, minY_d, maxX_d, maxY_d;
    while(iterator_stmt->execute())
    {
      *iterator_stmt >> minX_d >> minY_d >> maxX_d >> maxY_d;      
      if(std::get<0>(minX_d))
      {
        extent_min_x=std::get<1>(minX_d);
        extent_min_y=std::get<1>(minY_d);
        extent_max_x=std::get<1>(maxX_d);
        extent_max_y=std::get<1>(maxY_d);
      }
    }
      iterator_stmt->reset();
      delete iterator_stmt;
    return std::get<0>(minX_d);

  }
  
  std::pair<double, double> ChartsDBConnection::getRandomValidPointWithinExtent(double minX,double minY,double maxX,double maxY, std::string table) {

    // Create distributions to pick points from in double extent ranges
    std::uniform_real_distribution<> distX{minX, maxX};
    std::uniform_real_distribution<> distY{minY, maxY};
    double randX, randY;

    // Try until a point in the layer is returned
    do {
      randX=distX(eng);
      randY=distY(eng);
    } while(!isPointInLayer(randY, randX, table));

    return std::pair<double, double>(randX,randY);
  }

  bool ChartsDBConnection::isPointInLayer(double lat, double lon, std::string table, bool useSpatialIndex) {
    DUNE::Database::Statement* iterator_stmt;
    if(useSpatialIndex) {
      std::string c_stmt = "select sum(intersects(MakePoint(" + std::to_string(lon) + ", " + std::to_string(lat) + ", 4326), geom)) as c from (SELECT geom FROM " + table + " "
      "WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + table + "' AND "
          "search_frame = BuildMbr(" + std::to_string(lon) + ", " + std::to_string(lat) + "," + std::to_string(lon) + ", " + std::to_string(lat) + ", 4326)));";
      iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this); 
    } else {
      //std::string c_stmt = "select count(*) from " + table + " as d where intersects(GeomFromText(\"Point(" + std::to_string(lon) + " " + std::to_string(lat) + ")\", 4326), d.geom) limit 1";
      std::string c_stmt = "select count(*) from " + table + " as d where intersects(MakePoint(" + std::to_string(lon) + ", " + std::to_string(lat) + ", 4326), d.geom) limit 1";
      //std::cout << c_stmt << std::endl;
      iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this); 
    }

    std::pair<bool, int> intersections;

    iterator_stmt->execute();
    *iterator_stmt >> intersections;
    delete iterator_stmt;
    if(std::get<0>(intersections))
    {
      return std::get<1>(intersections);
    }
    return false;
  }
  double ChartsDBConnection::checkPlanMinDepth(const DUNE::IMC::PlanSpecification* plan, double currentLat, double curentLon, std::string table) {   
    std::string c_stmt = "select min(DRVAL1) from " + table + " as d where intersects(GeomFromText(\"" + planToWKT(plan, currentLat, curentLon) + "\", 4326), d.geom);";
    std::cout << c_stmt << std::endl;
    DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this); 

      std::pair<bool, double> DBDepth;

      while(iterator_stmt->execute())
      {
        *iterator_stmt >> DBDepth;
        
        if(std::get<0>(DBDepth))
        {
          iterator_stmt->reset();
          delete iterator_stmt;
          return std::get<1>(DBDepth); 
        }
      }
          iterator_stmt->reset();
          delete iterator_stmt;
      
    return -50.0;
  }
  bool ChartsDBConnection::checkPlanLanding(const DUNE::IMC::PlanSpecification* plan, double currentLat, double curentLon, std::string table) {
    std::string c_stmt = "select count(*), geom from " + table + " as l where intersects(GeomFromText(\"" + planToWKT(plan, currentLat, curentLon) + "\", 4326), l.geom)";
    //std::cout << c_stmt << std::endl;
    DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this); 

      std::pair<bool, int> DBDepth;

      while(iterator_stmt->execute())
      {
        *iterator_stmt >> DBDepth;
        
        if(std::get<0>(DBDepth))
        {
          iterator_stmt->reset();
          delete iterator_stmt;
          if(std::get<1>(DBDepth) > 0) {
            return true;
          } else {
            return false;
          }
        }
      }
          iterator_stmt->reset();
          delete iterator_stmt;
    return true;
  }

  std::string ChartsDBConnection::planToWKT(const DUNE::IMC::PlanSpecification* plan, double currentLat, double curentLon) {
    std::stringstream out;
    out.precision(15);
    out << "LineString(";
    if(currentLat >=0 && curentLon>=0) {
      out << std::setprecision(15) << DUNE::Math::Angles::degrees(curentLon) << " " << DUNE::Math::Angles::degrees(currentLat) << ", ";
    }
    // Iterate through plan maneuvers
    for (std::vector<DUNE::IMC::PlanManeuver*>::const_iterator itr = plan->maneuvers.begin(); itr != plan->maneuvers.end(); ++itr)
    {
      // For now just to GoTos.
      const DUNE::IMC::Goto* m = static_cast<const DUNE::IMC::Goto*>((*itr)->data.get());
      out << std::setprecision(15) << DUNE::Math::Angles::degrees(m->lon) << " " << DUNE::Math::Angles::degrees(m->lat);
      if(std::next(itr) != plan->maneuvers.end()) // Not last element
      {
        out << ", ";
      }
    }
    out << ")";
    return out.str();
  }

  bool ChartsDBConnection::findFirstIntersectingPoint(double startLat, double startLon, double endLat, double endLon, double &pointLat, double &pointLon, double &time, std::string table) {
    std::string c_stmt = "select Y(pnt), X(pnt), line_locate_point(lne, pnt) from (select lne, closestpoint(inter, makepoint(" + std::to_string(startLon) + ", " + std::to_string(startLat) + ", 4326)) as pnt from (select lne, intersection(geom, lne) as inter from " + table + " as l, (select makeline(makepoint(" + std::to_string(startLon) + ", " + std::to_string(startLat) + ", 4326), makepoint(" + std::to_string(endLon) + ", " + std::to_string(endLat) + ", 4326)) as lne) where isValid(inter) and inter not null))  order by distance(pnt, makepoint(" + std::to_string(startLon) + ", " + std::to_string(startLat) + ", 4326)) asc limit 1";
    DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *this); 
    //std::cout << c_stmt << std::endl;
    std::pair<bool, double> dbLat, dbLon, dbTime;

    while(iterator_stmt->execute()) // While there are results
    {
      *iterator_stmt >> dbLat >> dbLon >> dbTime;
      if(std::get<0>(dbLat))
      {
        pointLat = std::get<1>(dbLat);
        pointLon = std::get<1>(dbLon);
        time = std::get<1>(dbTime);
        iterator_stmt->reset();
        delete iterator_stmt;
        return true;
      } else {
        // TODO: Consider throwing some error here: NULL value returned
        std::cout << "PROBLEM in findFirstIntersect(): No dbLat"<< std::endl;
        std::cout << c_stmt << std::endl;
        iterator_stmt->reset();
        delete iterator_stmt;
        time=-1.0;
        return false;
      }
    }
    // Only executes when no intersection is found
    iterator_stmt->reset();
    delete iterator_stmt;

    return false;
  }
}

