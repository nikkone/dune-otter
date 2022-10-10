//***************************************************************************
// Copyright 2020-2021 Norwegian University of Science and Technology       *
// Department of Engineering Technology                                     *
//***************************************************************************
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

#ifndef ENCGIS_DBCONNECTION_HPP_INCLUDED
#define ENCGIS_DBCONNECTION_HPP_INCLUDED


// SQLITE3 headers.
#include <sqlite3/sqlite3.h>

#include <stdexcept>
namespace ENCGIS
{
  //! Database connection with functions for using Spatialite stored ENCs
  class DBconnection
  {
  public:

    /// @brief 
    class Error: public std::runtime_error
    {
    public:
      Error(std::string op, std::string msg):
        std::runtime_error("Error (" + op + "): " + msg)
      { }
    };

    /// @brief 
    /// @param filename 
    /// @param flag 
    /// @param SRIDin 
    DBconnection(std::string filename, int flag, int SRIDin);

    /// @brief 
    ~DBconnection();


    /// @brief 
    /// @param in_x 
    /// @param in_y 
    /// @param in_srid 
    /// @param out_x 
    /// @param out_y 
    /// @param out_srid 
    void transformSRID(double in_x, double in_y, unsigned in_srid, double &out_x, double &out_y, unsigned out_srid);

    /// @brief 
    /// @param sql_stmt 
    void runQuery(const std::string &sql_stmt);

    /// @brief 
    /// @param sql_stmt 
    /// @return 
    bool runNoOutputQuery(const std::string &sql_stmt);

    /// @brief 
    /// @param NotUsed 
    /// @param argc 
    /// @param argv 
    /// @param azColName 
    /// @return 
    static int callback(void *NotUsed, int argc, char **argv, char **azColName);

    /// @brief 
    void loadSpatialite();

    //! Checks whether a point lies within the the polygons of a layer.
    //! @param[in] lat The minimum longtitude in the extent, given in degrees.
    //! @param[in] lon The minimum latitude in the extent, given in degrees.
    //! @param[in] table The table in the database defining the layer.
    //! @param[in] useSpatialIndex If spatial index should be used before intersection. Default= true
    //! @return True if given point is in layer and false if not.
    bool isPointInLayer(double lat, double lon, std::string table="deparetable", bool useSpatialIndex=true);

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

    sqlite3 *db;
    int SRID;
  };
}

#endif //ENCGIS_HPP_INCLUDED


/*
#define SQLITE_OK           0    Successful result 
#define SQLITE_ERROR        1    Generic error 
#define SQLITE_INTERNAL     2    Internal logic error in SQLite 
#define SQLITE_PERM         3    Access permission denied 
#define SQLITE_ABORT        4    Callback routine requested an abort 
#define SQLITE_BUSY         5    The database file is locked 
#define SQLITE_LOCKED       6    A table in the database is locked 
#define SQLITE_NOMEM        7    A malloc() failed 
#define SQLITE_READONLY     8    Attempt to write a readonly database 
#define SQLITE_INTERRUPT    9    Operation terminated by sqlite3_interrupt()
#define SQLITE_IOERR       10    Some kind of disk I/O error occurred 
#define SQLITE_CORRUPT     11    The database disk image is malformed 
#define SQLITE_NOTFOUND    12    Unknown opcode in sqlite3_file_control() 
#define SQLITE_FULL        13    Insertion failed because database is full 
#define SQLITE_CANTOPEN    14    Unable to open the database file 
#define SQLITE_PROTOCOL    15    Database lock protocol error 
#define SQLITE_EMPTY       16    Internal use only 
#define SQLITE_SCHEMA      17    The database schema changed 
#define SQLITE_TOOBIG      18    String or BLOB exceeds size limit 
#define SQLITE_CONSTRAINT  19    Abort due to constraint violation 
#define SQLITE_MISMATCH    20    Data type mismatch 
#define SQLITE_MISUSE      21    Library used incorrectly 
#define SQLITE_NOLFS       22    Uses OS features not supported on host 
#define SQLITE_AUTH        23    Authorization denied 
#define SQLITE_FORMAT      24    Not used 
#define SQLITE_RANGE       25    2nd parameter to sqlite3_bind out of range 
#define SQLITE_NOTADB      26    File opened that is not a database file 
#define SQLITE_NOTICE      27    Notifications from sqlite3_log() 
#define SQLITE_WARNING     28    Warnings from sqlite3_log() 
#define SQLITE_ROW         100   sqlite3_step() has another row ready 
#define SQLITE_DONE        101   sqlite3_step() has finished executing 
*/