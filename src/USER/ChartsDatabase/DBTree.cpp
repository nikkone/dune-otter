//***************************************************************************
// Copyright 2020 Norwegian University of Science and Technology            *
// Department of Engineering Technology                                     *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

#include <USER/ChartsDatabase/DBTree.hpp>
#include <DUNE/Database/Statement.hpp>

namespace DUNE
{
  namespace ChartsDatabase
  {
    DBTree::DBTree(ChartsDatabase::ChartsDBConnection* dbcon)  : m_con(dbcon){
    }
    bool DBTree::resetTree(std::string tableName) {
      std::string c_stmt = "delete from " + tableName + ";";
      std::string c_stmt1 = "delete from sqlite_sequence where name='" + tableName + "';";

      //std::cout << c_stmt1 << std::endl;
      Database::Statement* iterator_stmt = new Database::Statement(c_stmt.c_str(), *m_con); 
      Database::Statement* iterator_stmt1 = new Database::Statement(c_stmt1.c_str(), *m_con);
      iterator_stmt->execute();
      iterator_stmt1->execute();
      delete iterator_stmt;
      delete iterator_stmt1;

      return true;
    }

    bool DBTree::createTree(std::string dbTreeName) {
      std::string c_stmt = "CREATE TABLE `" + dbTreeName + "` (`ID`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,`ParentID`	integer);";
      std::string c_stmt1 = "SELECT AddGeometryColumn('" + dbTreeName + "', 'geom', 4326, 'POINT', 'XY');";
      Database::Statement* iterator_stmt = new Database::Statement(c_stmt.c_str(), *m_con); 
      Database::Statement* iterator_stmt1 = new Database::Statement(c_stmt1.c_str(), *m_con);
      iterator_stmt->execute();
      iterator_stmt1->execute();
      delete iterator_stmt;
      delete iterator_stmt1;
      return true;
    }

    bool DBTree::deleteTree(std::string dbTreeName) {
      std::string c_stmt = "DROP TABLE `" + dbTreeName + "`;";
      Database::Statement* iterator_stmt = new Database::Statement(c_stmt.c_str(), *m_con); 
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
        Database::Statement* iterator_stmt = new Database::Statement(c_stmt.c_str(), *m_con); 
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
      Database::Statement* iterator_stmt2 = new Database::Statement(c_stmt2.c_str(), *m_con); 
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
}