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
                //! Size of the grid cells
                unsigned m_gridSize;
                //! Geometry type of grid, see ENCGIS::SearchGrid::gridtypes_t
                unsigned m_gridType;

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

                bind<IMC::PlanProbSpec>(this);
                }
                //! Update internal state with new parameter values.
                void
                onUpdateParameters(void)
                {
                    if(paramChanged(m_args.gridSize)) {
                        m_gridSize = m_args.gridSize;
                    }
                    if(paramChanged(m_args.gridType)) {
                        m_gridType = m_args.gridType;
                    }
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
                    m_searchGrid = new ENCGIS::SearchGrid(m_con);
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

                std::string polygonToEWKT(const IMC::MessageList<IMC::PolygonVertex> &polygon) {
                    std::string EWKT = "SRID=4326;POLYGON((";
                    for(IMC::MessageList<IMC::PolygonVertex>::const_iterator itr = polygon.begin();itr < polygon.end();itr++) {
                        //spew("lat %f, lon %f", (*itr)->lat, (*itr)->lon);
                        EWKT += std::to_string(DUNE::Math::Angles::degrees((*itr)->lon)) + " " + std::to_string(DUNE::Math::Angles::degrees((*itr)->lat)) + ",";
                    }
                    EWKT += std::to_string(DUNE::Math::Angles::degrees((*(polygon.begin()))->lon)) + " " + std::to_string(DUNE::Math::Angles::degrees((*(polygon.begin()))->lat)) + "))";
                    return EWKT;
                }

                void
                consume(const IMC::PlanProbSpec* msg)
                {
                    spew("Message received");
                    spew("Destination: %i", msg->getDestination());
                    spew("Problem Type%i", msg->problem_type);

                    // Only accept messages to this system
                    if (msg->getDestination() != getSystemId())
                    return;

                    /*if (msg->getDestinationEntity() != getEntityId())
                    return;*/

                    // Only accept feasible path problems
                    if (msg->problem_type != IMC::PlanProbSpec::TypeEnum::PPT_coverage)
                    return;

                    // Parse Custom Parameters
                    /*
                    Supported custom parameters:
                        a = [0,x], activate resulting plan
                        gg = Grid geometry
                        gs = Grid geometry edge size
                        p = Planner
                        paw = Planner azimuth weight
                        pdw = Planner distance weight
                    */
                    DUNE::Utils::TupleList custom = DUNE::Utils::TupleList(msg->custom);
                    std::map<std::string, std::string> custommap = custom.getMapReversed();

                    auto parameterit = custommap.find(std::string("a"));
                    bool activateResultingPlan= false;
                    if (parameterit != custommap.end()) {
                    try{
                        spew("Found a=%i", std::stoi(parameterit->second));
                        activateResultingPlan = (std::stoi(parameterit->second)) ? true : false;
                    } catch(...) {
                        err("Parameter \a\' not bool(int)");
                    }
                    }

                    parameterit = custommap.find(std::string("gg"));
                    if (parameterit != custommap.end()) {
                    spew("Found gg=%s", parameterit->second.c_str());
                    try{
                        m_gridType = std::stoul(parameterit->second);
                    } catch(...) {
                        err("Parameter \'gg\' not unsigned");
                    }
                    }

                    parameterit = custommap.find(std::string("gs"));
                    if (parameterit != custommap.end()) {
                    spew("Found gs=%s", parameterit->second.c_str());
                    try{
                        m_gridSize = std::stof(parameterit->second);
                    } catch(...) {
                        err("Parameter \'gs\' not float");
                    }
                    }
                    unsigned m_planner = 0;
                    float m_distanceWeight = 0;
                    float m_azimuthWeight = 0;
                    parameterit = custommap.find(std::string("p"));
                    if (parameterit != custommap.end()) {
                    spew("Found p=%s", parameterit->second.c_str());
                    try{
                        m_planner = std::stoi(parameterit->second);
                    } catch(...) {
                        err("Parameter \'p\' not unsigned");
                    }
                    }

                    parameterit = custommap.find(std::string("paw"));
                    if (parameterit != custommap.end()) {
                    spew("Found paw=%s", parameterit->second.c_str());
                    try{
                        m_azimuthWeight = std::stof(parameterit->second);
                    } catch(...) {
                        err("Parameter \'paw\' not unsigned");
                    }
                    }

                    parameterit = custommap.find(std::string("pdw"));
                    if (parameterit != custommap.end()) {
                    spew("Found pdw=%s", parameterit->second.c_str());
                    try{
                        m_distanceWeight = std::stof(parameterit->second);
                    } catch(...) {
                        err("Parameter \'pdw\' not unsigned");
                    }
                    }
                    // Convert from WGS-84 to EPSG32632
                    double start_northing, start_easting;
                    m_con->transformSRID(Math::Angles::degrees(msg->start_lon), Math::Angles::degrees(msg->start_lat), 4326, start_easting, start_northing, 32632);
                    spew("Planning start: %f, %f", start_easting, start_northing);

                    spew("Checking size");
                    if(msg->area.size() == 2) {
                        m_searchGrid->deleteGrid();
                        // Create planning bound
                        IMC::MessageList<IMC::PolygonVertex>::const_iterator itr = msg->area.begin();
                        for (unsigned i = 0; itr != msg->area.end(); ++itr, ++i)
                        {
                            spew("lat %f, lon %f", (*itr)->lat, (*itr)->lon);
                        }
                        itr = msg->area.begin();
                        double planningBounds[4];
                        m_con->transformSRID(Math::Angles::degrees((*itr)->lon), Math::Angles::degrees((*itr)->lat), 4326, planningBounds[0], planningBounds[1], 32632);
                        ++itr;
                        //m_con->transformSRID(Math::Angles::degrees(msg->end_lon), Math::Angles::degrees(msg->end_lat), 4326, end_easting, end_northing, 32632);
                        spew("Planning bounds:  %f, %f, %f, %f", planningBounds[0], planningBounds[2], planningBounds[1], planningBounds[3]);
                        m_con->transformSRID(Math::Angles::degrees((*itr)->lon), Math::Angles::degrees((*itr)->lat), 4326, planningBounds[2], planningBounds[3], 32632);
                        m_searchGrid->createGrid(planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3], m_gridSize, ENCGIS::SearchGrid::gridtypes_t(m_gridType));
                        m_searchGrid->setGridWeightsFromLandDistance();
                    } else if(msg->area.size() > 2) {
                        m_searchGrid->deleteGrid();
                        m_searchGrid->createGrid(polygonToEWKT(msg->area), m_gridSize, ENCGIS::SearchGrid::gridtypes_t(m_gridType));
                        debug("Grid Created from EKWT");
                        m_searchGrid->setGridWeightsFromLandDistance();
                        spew("Weights of grid set");
                    } else {
                        spew("Polygon too small.");
                        return;
                    }
                    m_searchGrid->normalizeWeights(true);




                    // Start time for computation measurments.
                    auto start = std::chrono::high_resolution_clock::now();

                    // Find coverage path
                    int cell = m_searchGrid->getClosestCell(start_easting, start_northing);
                    std::vector<int> cells;
                    float initialAzimuth = 0.0;
                    switch (m_planner)
                    {
                    case 0:
                        cells = m_searchGrid->calculateSearchPathDistance(cell);
                        break;
                    case 1:
                        /* code */
                        return;
                        break;
                    case 2:
                        /* code */
                        return;
                        break;
                    case 3:
                        cells = m_searchGrid->calculateSearchPathAzimuth(cell, initialAzimuth, m_azimuthWeight);
                        break;
                    case 4:
                        /* code */
                        return;
                        break;
                    case 5:
                        /* code */
                        return;
                        break;
                    case 6:
                        /* code */
                        return;
                        break; 
                    case 7:
                        cells = m_searchGrid->calculateSearchPathGlobal(cell, initialAzimuth, m_azimuthWeight,m_distanceWeight);
                        break;                  
                    default:
                        break;
                    }
                    //std::vector<int> cells = m_searchGrid->calculateSearchPathAzimuth(cell, 0.0, 0.25);
                    //std::vector<int> cells = m_searchGrid->calculateSearchPathGlobal(cell, 0.0, 0.1,0.0003);

                    //std::vector<int> cells = m_searchGrid->calculateSearchPathDistance(cell);


                    // End time for computation time measurement
                    auto stop1 = std::chrono::high_resolution_clock::now();
                    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start);
                    std::cout << "Path found in: "
                    << duration1.count() << " microseconds" << std::endl;
                    

                    // Remove redundant cells from path in order to reduce plan size
                     std::vector<int> rcells = m_searchGrid->removeRedundantCells(cells);
                    // Create vector of path waypoints
                    auto planVec = m_searchGrid->locationsFromCells(rcells);
                    debug("Got locations from cells");

                    // Write plan to spatialite DBTree, not needed for functionality
                    ENCGIS::DBconnection* m_writable = new ENCGIS::DBconnection(m_args.resultsDBpath, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 32632);
                    ENCGIS::DBTree* tree = new ENCGIS::DBTree(m_writable);
                    m_writable->runNoOutputQuery("select InitSpatialMetadata(1);");
                    tree->resetTree("tree");
                    tree->createTree("tree");
                    tree->resetTree("rtree");
                    tree->createTree("rtree");
                    auto planVec32632 = m_searchGrid->locationsFromCells(cells, 32632);
                    auto rplanVec32632 = m_searchGrid->locationsFromCells(rcells, 32632);
                    tree->pathToDBTree("tree", planVec32632);
                    tree->pathToDBTree("rtree", rplanVec32632);
                    inf("Wrote to tree");
                    Memory::clear(tree);
                    Memory::clear(m_writable);

                    // Set weights in grid for visualization purposes, not needed for functionality
                    m_searchGrid->setGridWeightsFromLandDistance();
                    m_searchGrid->normalizeWeights(true);

                    // Turn vector of waypoints into a IMC::PlanDB and dispatch/submit it to the plan database
                    IMC::PlanDB pdb = createPlanDBEntry(planVec, "autoPlan", 1.0);
                    debug("IMC plan created");
                    
                    // Check if path is too long to use
                    unsigned total = pdb.getSerializationSize();
                    if (total > DUNE_IMC_CONST_MAX_SIZE) {
                        err("Path IMC message too long, cannot be activated.(%u > %u)", total, DUNE_IMC_CONST_MAX_SIZE);
                        return;
                    }
                    dispatch(pdb);
                    debug("IMC plan dispatched");

                    // Activate the created plan if a=1 in custom parameters
                    if(activateResultingPlan) {
                        activatePlan("autoPlan");
                        debug("Plan activated");
                    }

                }

                void
                sequentialPlan(std::string plan_id, const DUNE::IMC::MessageList<DUNE::IMC::Maneuver>* maneuvers, DUNE::IMC::PlanSpecification& result)
                {
                    DUNE::IMC::PlanManeuver last_man;

                    DUNE::IMC::MessageList<DUNE::IMC::Maneuver>::const_iterator itr;
                    unsigned i = 0;
                    for (itr = maneuvers->begin(); itr != maneuvers->end(); itr++, i++)
                    {
                    if (*itr == NULL)
                        continue;

                    DUNE::IMC::PlanManeuver man_spec;

                    man_spec.data.set(*(*itr));
                    man_spec.maneuver_id = DUNE::Utils::String::str(i + 1);
                    if (itr == maneuvers->begin())
                    {
                        // no transitions.
                    }
                    else
                    {
                        DUNE::IMC::PlanTransition trans;
                        trans.conditions = "ManeuverIsDone";
                        trans.dest_man = man_spec.maneuver_id;
                        trans.source_man = last_man.maneuver_id;

                        result.transitions.push_back(trans);
                    }

                    result.maneuvers.push_back(man_spec);

                    last_man = man_spec;
                    }

                    result.plan_id = plan_id;
                    result.start_man_id = "1";
                }

                DUNE::IMC::PlanDB createPlanDBEntry(const std::vector<std::pair<double, double>> &planVec, std::string plan_id, fp32_t speed) {

                DUNE::IMC::MessageList<DUNE::IMC::Maneuver> maneuvers; //Define list of meneuvers

                    // Make maneuvers
                    for(auto i = planVec.begin(); i < planVec.end();i++) {
                        //inf("%f, %f", i->first, i->second);
                        DUNE::IMC::Goto* go_near = new DUNE::IMC::Goto();
                        go_near->lat = DUNE::Math::Angles::radians(i->second);
                        go_near->lon = DUNE::Math::Angles::radians(i->first);
                        go_near->speed_units = DUNE::IMC::SUNITS_METERS_PS;
                        go_near->speed = speed;
                        maneuvers.push_back(*go_near);

                        delete go_near;
                    }
                    DUNE::IMC::PlanSpecification pspec;
                    sequentialPlan(plan_id, &maneuvers, pspec);
                    DUNE::IMC::PlanDB pdb;
                    pdb.op = DUNE::IMC::PlanDB::DBOP_SET;
                    pdb.type = DUNE::IMC::PlanDB::DBT_REQUEST;
                    pdb.plan_id = pspec.plan_id;
                    pdb.arg.set(pspec);
                    pdb.request_id = 0;

                    return pdb;
                }

                void activatePlan(std::string plan_id) {
                    bool ignore_errors = true;
                    IMC::PlanControl pcontrol;
                    pcontrol.type = IMC::PlanControl::PC_REQUEST;
                    pcontrol.op = IMC::PlanControl::PC_START;
                    pcontrol.plan_id = plan_id;
                    pcontrol.setDestination(m_ctx.resolver.id());
                    if (ignore_errors)
                    pcontrol.flags = IMC::PlanControl::FLG_IGNORE_ERRORS;
                    dispatch(pcontrol);
                    spew("Plan start request sent");
                }

                void
                onMain(void)
                {

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
