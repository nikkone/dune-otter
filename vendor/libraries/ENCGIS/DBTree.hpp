//***************************************************************************
// Copyright 2020-2021 Norwegian University of Science and Technology       *
// Department of Engineering Technology                                     *
//***************************************************************************
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

#ifndef ENCGIS_DBTREE_HPP_INCLUDED
#define ENCGIS_DBTREE_HPP_INCLUDED

// ISO C++ 98 headers.
#include <string>

// DUNE headers.
#include "Connection.hpp"
namespace ENCGIS
{
  //! A class with functions for creating and modifying a tree in a Spatialite database.
  //! Mainly used for visualizing the tree in GIS. Example query for visualizing tree:
  //! "select ID, makeLine((select geom from tree where ID = t.ParentID), t.geom) from tree as t"
  class DBTree {
      public:
      //! Constructor
      //! @param[in] dbcon A connection to the Spatialite database where changes are done.
      DBTree(ENCGIS::ChartsDBConnection* dbcon);

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
      ENCGIS::ChartsDBConnection* m_con;
  };
}

#endif //ENCGIS_DBTREE_HPP_INCLUDED
