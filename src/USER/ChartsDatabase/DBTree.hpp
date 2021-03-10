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

#ifndef DUNE_CHARTSDATABASE_DBTREE_HPP_INCLUDED_
#define DUNE_CHARTSDATABASE_DBTREE_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <string>

// DUNE headers.
#include <USER/ChartsDatabase/Connection.hpp>
namespace DUNE
{
  namespace ChartsDatabase
  {
    //! A class with functions for creating and modifying a tree in a Spatialite database.
    //! Mainly used for visualizing the tree in GIS. Example query for visualizing tree:
    //! "select ID, makeLine((select geom from tree where ID = t.ParentID), t.geom) from tree as t"
    class DBTree {
        public:
        //! Constructor
        //! @param[in] dbcon A connection to the Spatialite database where changes are done.
        DBTree(ChartsDatabase::ChartsDBConnection* dbcon);

        //! Creates a table for storing a tree structure in the database.
        //! @param[in] dbTreeName The table in the database to use.
        //! @return TODO: true if success, false if not.
        bool createTree(std::string dbTreeName);

        //! Deletes a table in the database.
        //! @param[in] dbTreeName The table in the database to use.
        //! @return TODO: true if success, false if not.
        bool deleteTree(std::string dbTreeName);

        //! Remover the contents of a table used for storing the tree structure in the database.
        //! @param[in] dbTreeName The table in the database to use.
        //! @return TODO: true if success, false if not.
        bool resetTree(std::string dbTreeName);

        //! Inserts a node in the tree stored in the database.
        //! @param[in] dbTreeName The table in the database to use.
        //! @param[in] ParentID The parent of the node. For root node, this is set equalt to ID.
        //! @param[in] lat The longitudinal position of the node.
        //! @param[in] lon The latitudinal position of the node.
        //! @param[in] ID A unique ID for the node. Auto-increments if ID=0
        //! @return TODO: true if success, false if not.
        bool insertNode(std::string dbTreeName, unsigned ParentID,double lat, double lon, unsigned ID=0);

        //! Gets the location of a node in the tree stored in the database.
        //! @param[in] dbTreeName The table in the database to use.
        //! @param[in] ID The ID whos location is returned
        //! @return The position of the node as a longtitude, latitude pair in degrees.
        std::pair<double, double> getNodeLocation(std::string dbTreeName, unsigned ID);

        protected:
        ChartsDatabase::ChartsDBConnection* m_con;
    };
  }
}

#endif
