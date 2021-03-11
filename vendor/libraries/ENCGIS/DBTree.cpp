//***************************************************************************
// Copyright 2020-2021 Norwegian University of Science and Technology       *
// Department of Engineering Technology                                     *
//***************************************************************************
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

#include "DBTree.hpp"
#include <DUNE/Database/Statement.hpp>

namespace ENCGIS
{
  DBTree::DBTree(ENCGIS::ChartsDBConnection* dbcon)  : m_con(dbcon){
  }
  bool DBTree::resetTree(std::string tableName) {
    std::string c_stmt = "delete from " + tableName + ";";
    std::string c_stmt1 = "delete from sqlite_sequence where name='" + tableName + "';";

    //std::cout << c_stmt1 << std::endl;
    DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *m_con); 
    DUNE::Database::Statement* iterator_stmt1 = new DUNE::Database::Statement(c_stmt1.c_str(), *m_con);
    iterator_stmt->execute();
    iterator_stmt1->execute();
    delete iterator_stmt;
    delete iterator_stmt1;

    return true;
  }

  bool DBTree::createTree(std::string dbTreeName) {
    std::string c_stmt = "CREATE TABLE `" + dbTreeName + "` (`ID`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,`ParentID`	integer);";
    std::string c_stmt1 = "SELECT AddGeometryColumn('" + dbTreeName + "', 'geom', 4326, 'POINT', 'XY');";
    DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *m_con); 
    DUNE::Database::Statement* iterator_stmt1 = new DUNE::Database::Statement(c_stmt1.c_str(), *m_con);
    iterator_stmt->execute();
    iterator_stmt1->execute();
    delete iterator_stmt;
    delete iterator_stmt1;
    return true;
  }

  bool DBTree::deleteTree(std::string dbTreeName) {
    std::string c_stmt = "DROP TABLE `" + dbTreeName + "`;";
    DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *m_con); 
    iterator_stmt->execute();
    delete iterator_stmt;
    return false;
  }

  bool DBTree::insertNode(std::string tableName, unsigned ParentID,double lat, double lon, unsigned ID) {
    std::string IDStr;
    if(ID==0) {
      IDStr="NULL";
    } else {
      IDStr=std::to_string(ID);
    }
    std::string c_stmt = "insert into " + tableName + " values(" + IDStr + "," + std::to_string(ParentID) + ",MakePoint(" + std::to_string(lon) + ", " + std::to_string(lat) + ",4326 ));";
    //std::cout << c_stmt << std::endl;
    try{
      DUNE::Database::Statement* iterator_stmt = new DUNE::Database::Statement(c_stmt.c_str(), *m_con); 
      iterator_stmt->execute();
      iterator_stmt->reset();
      delete iterator_stmt;
      return true;
    } catch(std::runtime_error& e) {
      return false;
    }
  }
  std::pair<double, double> DBTree::getNodeLocation(std::string tableName, unsigned ID) {
    std::string c_stmt2 = "select X(geom), Y(geom) from (select geom from " + tableName + " where ID="+std::to_string(ID)+")";
    //std::cout << c_stmt2 << std::endl;


    std::pair<bool, double> lon,lat;
    DUNE::Database::Statement* iterator_stmt2 = new DUNE::Database::Statement(c_stmt2.c_str(), *m_con); 
    iterator_stmt2->execute();
    *iterator_stmt2 >> lon >> lat;
    iterator_stmt2->reset();
    delete iterator_stmt2;
    if(!std::get<0>(lon))
    {
      std::cout << "Node not found." << std::endl;
      //break;
      // TODO: Trow exeption
    }
    //std::cout << std::to_string(std::get<1>(lat)) << std::to_string(std::get<1>(lon)) << std::endl;
    return std::pair<double, double>(std::get<1>(lat),std::get<1>(lon));
  }
}