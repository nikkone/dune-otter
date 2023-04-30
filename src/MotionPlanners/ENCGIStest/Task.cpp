//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
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
// C++ program to find out execution time of
// of functions
#include <algorithm>
#include <chrono>
#include <iostream>
#include<vector>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// ENC database to use with the OMPL integration for DUNE
#include <ENCGIS/DBconnection.hpp>
#include <ENCGIS/DBTree.hpp>

// OMPL integration for DUNE
#include <OMPL/setup.hpp>
#include <OMPL/OMPLfunctions.hpp>
#include <OMPL/OMPLMotionValidator2.hpp>
#include <OMPL/OMPLMotionValidator3.hpp>

namespace MotionPlanners
{
  //! This task demonstrates how OMPL is used in DUNE along with ENCGIS
  //! @author Nikolai Lauvås
  namespace ENCGIStest
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
      //! How long a planner is run before terminated.
      double maxPlaningTime;
      //! If a valid path is available, allow optimizing until the given time. If non-optimizing planner used, this is ignored.
      double minPlaningTime;
      //! Defines the bounds of the area the path planner operates on.
      std::vector<double> planningBounds;
      //! Defines the start and end point to use while developing
      std::vector<double> startAndEnd;
    };
    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Database connection
      ENCGIS::DBconnection* m_con;
      ENCGIS::isPointInLayerStatement       *pointCheck;
      ENCGIS::lineIntersectLayerStatement *lineCheck;
      ENCGIS::getClosestIntersectWithOffset *lineCheck2;
      bool m_intermediate;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_con(NULL)
      {
        param("DB Path", m_args.dbPath)
        .defaultValue("")
        .description("Path of the db");
      
        param("Result DB Path", m_args.resultsDBpath)
        .defaultValue("")
        .description("If set, the results are written to trees in this db.");

        param("DB Path", m_args.dbPath)
        .defaultValue("")
        .description("Path of the db");

        param("Navigable Layer Name", m_args.dbNavigableLayerName)
        .defaultValue("navigable")
        .description("Navigable Layer Name");

        param("Innavigable Layer Name", m_args.dbInnavigableLayerName)
        .defaultValue("innavigable")
        .description("Innavigable Layer Name");

        param("Max Planning Time", m_args.maxPlaningTime)
        .units(DUNE::Units::Second)
        .defaultValue("60.0")
        .description("How long a planner is run before terminated");

        param("Min Planning Time", m_args.minPlaningTime)
        .units(DUNE::Units::Second)
        .defaultValue("10.0")
        .description("If a valid path is available, allow optimizing until the given time. If non-optimizing planner used, this is ignored.");

        param("Planning Bounds", m_args.planningBounds)
        .size(4)
        .defaultValue("568399.476507, 7031678.685762, 571101.488332, 7038044.467683")
        .description("Define the area searched for a solution (minLat, minLon, maxLat, maxLon)");

        param("Start and Goal", m_args.startAndEnd)
        .size(4)
        .defaultValue("569142.113652, 7035964.208531, 569354.798021, 7032506.975707")
        .description("A starting point and end point to use while developing");   
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        try{
          m_con = new ENCGIS::DBconnection(m_args.dbPath, SQLITE_OPEN_READONLY, 32632);
        } catch(std::runtime_error& e) {
          err(DTR("Problem opening charts database: %s"), e.what());
          setEntityState(IMC::EntityState::ESTA_FAULT, Status::CODE_MISSING_DATA);
        }
        try{
          pointCheck = new ENCGIS::isPointInLayerStatement(m_args.dbNavigableLayerName, "geometry", m_con->db, 32632);
        } catch(std::runtime_error& e) {
          err(DTR("Problem creating query for navigable layer: %s"), e.what());
          setEntityState(IMC::EntityState::ESTA_FAULT, Status::CODE_MISSING_DATA);
        }

        try{
          lineCheck = new ENCGIS::lineIntersectLayerStatement(m_args.dbInnavigableLayerName, "geometry", m_con->db, 32632);
        } catch(std::runtime_error& e) {
          err(DTR("Problem creating query for innavigable layer: %s"), e.what());
          setEntityState(IMC::EntityState::ESTA_FAULT, Status::CODE_MISSING_DATA);
        }
        try{
          lineCheck2 = new ENCGIS::getClosestIntersectWithOffset(m_args.dbInnavigableLayerName, "geometry", m_con->db, 32632);
        } catch(std::runtime_error& e) {
          err(DTR("Problem creating query for innavigable layer: %s"), e.what());
          setEntityState(IMC::EntityState::ESTA_FAULT, Status::CODE_MISSING_DATA);
        }
                  
          
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        // Set OMPL to use the console output of this task
        ompl::msg::OutputHandler *oh = new OMPLforDUNE::OutputHandlerDUNEConsole(this);
        ompl::msg::useOutputHandler(oh);
        ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV2);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        try {
         Memory::clear(m_con);
        }
        catch(std::runtime_error& e) {
          err(DTR("Could not clear charts database class: %s"), e.what());
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        // Create bounds
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0,m_args.planningBounds[1]);
        bounds.setHigh(0,m_args.planningBounds[3]);
        bounds.setLow(1,m_args.planningBounds[0]);
        bounds.setHigh(1,m_args.planningBounds[2]);
        auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    // Define bounds of searching space
        space->setBounds(bounds);

    // Define a simple setup class
        ompl::geometric::SimpleSetup ss(space);
        ompl::base::MotionValidatorPtr mp2 = std::make_shared<OMPLintegrationENCGIS::ChartsDBMotionValidator2>(ss.getSpaceInformation(), lineCheck, 4);
        ompl::base::MotionValidatorPtr mp3 = std::make_shared<OMPLintegrationENCGIS::ChartsDBMotionValidator3>(ss.getSpaceInformation(), lineCheck,lineCheck2, 4);


        ob::State *a = ss.getSpaceInformation()->allocState();
        const auto statea= static_cast<ompl::base::RealVectorStateSpace::StateType *>(a);
        statea->values[0] = m_args.startAndEnd[1];
        statea->values[1] = m_args.startAndEnd[0];
        std::cout << "State a: " << statea->values[0] << " " << statea->values[1] << std::endl;
        ob::State *b = ss.getSpaceInformation()->allocState();
        const auto stateb= static_cast<ompl::base::RealVectorStateSpace::StateType *>(b);
        stateb->values[0] = m_args.startAndEnd[3];
        stateb->values[1] = m_args.startAndEnd[2];
        std::cout << "State b: " << stateb->values[0] << " " << stateb->values[1] << std::endl;


        //    double X=0, Y = 0;
        // Get starting timepoint
        auto start = std::chrono::high_resolution_clock::now();
        bool res = mp2->checkMotion(statea, stateb);
        auto stop1 = std::chrono::high_resolution_clock::now();
        bool res2 = mp3->checkMotion(statea, stateb);
        //double result = lineCheck2->run(m_args.startAndEnd[0],m_args.startAndEnd[1],m_args.startAndEnd[2],m_args.startAndEnd[3], X, Y);
        // Get ending timepoint
        auto stop2 = std::chrono::high_resolution_clock::now();

        ss.getSpaceInformation()->freeState(a);
        ss.getSpaceInformation()->freeState(b);
        if(res) {inf("acc");}
        if(res2) {inf("acc2");}
        // Get duration. Substart timepoints to 
        // get durarion. To cast it to proper unit
        // use duration cast method
        auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start);
        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - stop1);
        std::cout << "Time taken by function1: "
         << duration1.count() << " microseconds" << std::endl;// << "X,Y: " << X<< "," << Y << "," << result << std::endl;
         std::cout << "Time taken by function2: "
         << duration2.count() << " microseconds" << std::endl;


        while (!stopping())
        {
          waitForMessages(1.5);
        }
      }
    };
  }
}

DUNE_TASK
