//***************************************************************************
// Copyright 2013-2021 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
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
 
// DUNE headers.
#include <DUNE/DUNE.hpp>

// Additional headers
#include <ENCGIS/DBconnection.hpp>
#include <ENCGIS/SearchGrid.hpp>
#include <algorithm>
#include <chrono>

namespace Control
{
    namespace Search
    {
        namespace Greedy
        {
            using DUNE_NAMESPACES;

            struct Arguments
            {
                //! The path of the database.
                std::string dbPath;
                std::string resultsDBpath;
                //! Navigable Layer Name
                std::string dbNavigableLayerName;
                //! Innavigable Layer Name
                std::string dbInnavigableLayerName;
                //! Defines the bounds of the area the search path planner operates on.
                std::vector<double> planningBounds;
                //! Defines the start and end point to use while developing
                std::vector<double> start;
                //! Size of the grid cells
                unsigned gridSize;
                //! Geometry type of grid, see ENCGIS::SearchGrid::gridtypes_t
                unsigned gridType;
            };

            struct Task: public DUNE::Tasks::Task
            {
                //! Task arguments.
                Arguments m_args;
                //! Database connection
                ENCGIS::DBconnection* m_con;
                ENCGIS::SearchGrid* m_searchGrid;


                Task(const std::string& name, Tasks::Context& ctx):
                    DUNE::Tasks::Task(name, ctx),
                    m_con(NULL),
                    m_searchGrid(NULL)
                {
                param("DB Path", m_args.dbPath)
                .defaultValue("")
                .description("Path of the db");

                param("Result DB Path", m_args.resultsDBpath)
                .defaultValue("")
                .description("If set, the results are written to trees in this db.");

                param("Navigable Layer Name", m_args.dbNavigableLayerName)
                .defaultValue("navigable")
                .description("Navigable Layer Name");

                param("Innavigable Layer Name", m_args.dbInnavigableLayerName)
                .defaultValue("innavigable")
                .description("Innavigable Layer Name");

                param("Grid Size", m_args.gridSize)
                .defaultValue("100")
                .description("Size of the grid cells");

                param("Grid Type", m_args.gridType)
                .defaultValue("0")
                .description("Geometry type of grid, 0=HEX, 1=Square, 2=Triangular.");

                param("Planning Bounds", m_args.planningBounds)
                .size(4)
                .defaultValue("568399.476507, 7031678.685762, 571101.488332, 7038044.467683")
                .description("Define the area searched for a solution (minLat, minLon, maxLat, maxLon)");

                param("Start", m_args.start)
                .size(2)
                .defaultValue("569142.113652, 7035964.208531")
                .description("A starting point to use while developing");


                }
                //! Update internal state with new parameter values.
                void
                onUpdateParameters(void)
                {

                }
                void
                onResourceAcquisition(void)
                {
                    try{
                    m_con = new ENCGIS::DBconnection(m_args.dbPath, SQLITE_OPEN_READWRITE, 32632);
                    } catch(std::runtime_error& e) {
                    err(DTR("Problem opening charts database: %s"), e.what());
                    // Set task state to failure
                    }
                    m_searchGrid = new ENCGIS::SearchGrid(m_con->db);
                }


                void
                onResourceRelease(void) {
                    try {
                        Memory::clear(m_con);
                        Memory::clear(m_searchGrid);
                    }
                        catch(std::runtime_error& e) {
                        err(DTR("Could not clear charts database class: %s"), e.what());
                    }
                }

                void
                onResourceInitialization(void)
                {
                }



                void
                onMain(void)
                {
                    inf("Bounds: %f %f - %f %f", m_args.planningBounds[0], m_args.planningBounds[1], m_args.planningBounds[2], m_args.planningBounds[3]);
                    inf("Start: %f %f", m_args.start[0], m_args.start[1]);
                    auto start = std::chrono::high_resolution_clock::now();
                    m_searchGrid->createGrid(m_args.planningBounds[0], m_args.planningBounds[1], m_args.planningBounds[2], m_args.planningBounds[3], m_args.gridSize, ENCGIS::SearchGrid::gridtypes_t(m_args.gridType));

                    int cell = m_searchGrid->getClosestCell(m_args.start[0], m_args.start[1]);
                    std::vector<int> cells;
                    
                    // Greedy algorithm
                    while(cell != 0) {
                        cells.push_back(cell);
                        m_searchGrid->setCellWeight(cell,-1);
                        cell = m_searchGrid->getLocalOptimalNeighbour(cell);
                        if(cell == 0) {
                            cell = m_searchGrid->getClosestUnsearchedCell(cells.back());
                        }
                        //inf("Cell: %d", cell);
                    }

                    auto stop1 = std::chrono::high_resolution_clock::now();
                    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start);
                    std::cout << "Time taken by function1: "
                    << duration1.count() << " microseconds" << std::endl;

                    auto planVec = m_searchGrid->locationsFromCells(cells);
                    /*inf("Optimal neighbor: %d", m_searchGrid->getLocalOptimalNeighbour(414));
                    for(auto i = planVec.begin(); i < planVec.end();i++) {
                        inf("%f, %f", i->first, i->second);
                    }*/

          ENCGIS::DBconnection* m_writable = new ENCGIS::DBconnection(m_args.resultsDBpath, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 32632);
          ENCGIS::DBTree* tree = new ENCGIS::DBTree(m_writable);
          m_writable->runNoOutputQuery("select InitSpatialMetadata(1);");
          tree->resetTree("tree");
          tree->createTree("tree");
          m_searchGrid->pathToDBTree(planVec, "tree", tree);
          inf("Wrote to tree");
          Memory::clear(tree);
          Memory::clear(m_writable);
                    while(!stopping()) {

                    //consumeMessages();
                    waitForMessages(1.0);
                    }
                    m_searchGrid->deleteGrid();
                }
            };
        }
    }
}
DUNE_TASK
