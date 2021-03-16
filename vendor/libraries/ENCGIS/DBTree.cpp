//***************************************************************************
// Copyright 2020-2021 Norwegian University of Science and Technology       *
// Department of Engineering Technology                                     *
//***************************************************************************
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

#include "DBTree.hpp"

namespace ENCGIS
{
  DBTree::DBTree(ENCGIS::DBconnection* dbcon)  : m_con(dbcon){
  }
  bool DBTree::resetTree(std::string tableName) {
    std::string c_stmt = "delete from " + tableName + ";";
    std::string c_stmt1 = "delete from sqlite_sequence where name='" + tableName + "';";

    bool result_c_stmt = m_con->runNoOutputQuery(c_stmt);
    bool result_c_stmt1 = m_con->runNoOutputQuery(c_stmt1);
    return result_c_stmt1 || result_c_stmt;
  }

  bool DBTree::createTree(std::string dbTreeName) {
    std::string c_stmt = "CREATE TABLE `" + dbTreeName + "` (`ID`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,`ParentID`	integer);";
    std::string c_stmt1 = "SELECT AddGeometryColumn('" + dbTreeName + "', 'geom', 4326, 'POINT', 'XY');";
    bool result_c_stmt = m_con->runNoOutputQuery(c_stmt);
    bool result_c_stmt1 = m_con->runNoOutputQuery(c_stmt1);
    return result_c_stmt1 || result_c_stmt;
  }

  bool DBTree::deleteTree(std::string dbTreeName) {
    std::string c_stmt = "select DropGeoTable('" + dbTreeName + "');";
    return m_con->runNoOutputQuery(c_stmt);
  }
  bool DBTree::insertNode(std::string tableName, unsigned ParentID,double lat, double lon, unsigned ID) {
    std::string IDStr;
    if(ID==0) {
      IDStr="NULL";
    } else {
      IDStr=std::to_string(ID);
    }
    std::string c_stmt = "insert into " + tableName + " values(" + IDStr + "," + std::to_string(ParentID) + ",MakePoint(" + std::to_string(lon) + ", " + std::to_string(lat) + ",4326 ));";
    return m_con->runNoOutputQuery(c_stmt);
  }
  
  std::pair<double, double> DBTree::getNodeLocation(std::string tableName, unsigned ID) {
    std::string sql_stmt = "select X(geom), Y(geom) from (select geom from " + tableName + " where ID="+std::to_string(ID)+")";
  
    //Setup
    sqlite3_stmt* m_handle;
    
    if (sqlite3_prepare_v2(m_con->db, sql_stmt.c_str(), sql_stmt.length(), &m_handle, 0) != SQLITE_OK)
    {
      ENCGIS::DBconnection::Error("Failed to prepare statement", sql_stmt.c_str());
    }
    std::pair<double, double> ret;
    // Execute
    if(sqlite3_step(m_handle) == SQLITE_ROW) {
      int m_idx = 0;
      if (sqlite3_column_type(m_handle, m_idx) != SQLITE_FLOAT)
        throw ENCGIS::DBconnection::Error("column result is not of FLOAT type", "");
      ret.first = sqlite3_column_double(m_handle, m_idx++);
      if (sqlite3_column_type(m_handle, m_idx) != SQLITE_FLOAT)
        throw ENCGIS::DBconnection::Error("column result is not of FLOAT type", "");
      ret.second = sqlite3_column_double(m_handle, m_idx);
    } else {
        throw ENCGIS::DBconnection::Error("Could not execute", sql_stmt);
    }

    // Teardown
    if (m_handle)
      sqlite3_finalize(m_handle);
    return ret;
  }
}