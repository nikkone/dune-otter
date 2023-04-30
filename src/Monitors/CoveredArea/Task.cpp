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
#include <ENCGIS/SearchGridCoverageState.hpp>

namespace Monitors
{
  //! This task updates a gridded map showing where a vehicle system has searched. 
  //! TODO: Detection Model
  //! TODO: Decrease searched state with time
  //! @author Nikolai Lauvås
  namespace CoveredArea
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      std::string gridDBpath;

      float initialSensorRange;
      std::vector<std::string> otherVehicles;
      //! Size of the grid cells
      unsigned gridSize;
      //! Geometry type of grid, see ENCGIS::SearchGrid::gridtypes_t
      unsigned gridType;
      //! Defines the bounds of the area the path planner operates on.
      std::vector<double> planningBounds;

      float timestepConstantDecrease;
      float timestepFactorDecrease;

    };
    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments.
      Arguments m_args;
      //! Database connection
      ENCGIS::DBconnection* m_con;
      ENCGIS::SearchGridCoverageState* m_searchGridCoverage;

      std::map<uint16_t, std::pair<double,double>> pendingUpdates;

      std::vector<uint16_t> monitoredVehicles;

      unsigned rpm;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_con(NULL),
        m_searchGridCoverage(NULL)
      {
        param("Other Vehicles", m_args.otherVehicles)
        .description("The source/vehicle names of other entities in the system.")
        .defaultValue("");

        param("Initial Sensor Range", m_args.initialSensorRange)
        .description("The initial maximum range of the sensor")
        .defaultValue("50.0");  

        param("Timestep Grid Constant Decrease", m_args.timestepConstantDecrease)
        .defaultValue("0.0")
        .description("The value substracted from each grid cell at each timestep.");

        param("Timestep Grid Factor Decrease", m_args.timestepFactorDecrease)
        .defaultValue("1.0")
        .description("The value multiplied with each grid cell at each timestep. [0.0, 1.0]");

        param("Grid DB Path", m_args.gridDBpath)
        .defaultValue("")
        .description("The path of a DB to write the grid in.");

        param("Grid Size", m_args.gridSize)
        .defaultValue("100")
        .description("Size of the grid cells");

        param("Grid Type", m_args.gridType)
        .defaultValue("0")
        .description("Geometry type of grid, 0=HEX, 1=Square, 2=Triangular.");

        param("Planning Bounds", m_args.planningBounds)
        .size(4)
        .defaultValue("568164, 7034500, 571101, 7038044")
        .description("Define the area searched for a solution (minLat, minLon, maxLat, maxLon)");


        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        bind<IMC::EstimatedState>(this);
        bind<IMC::Rpm>(this);

      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.otherVehicles)) {
          monitoredVehicles.clear();
          for(auto iter = m_args.otherVehicles.begin(); iter != m_args.otherVehicles.end(); iter++) {
            monitoredVehicles.push_back(resolveSystemName(*iter));
          }
          for(auto iter = monitoredVehicles.begin(); iter != monitoredVehicles.end(); iter++) {
            inf("%u", *iter);
          }
        }
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
          m_con = new ENCGIS::DBconnection(m_args.gridDBpath, SQLITE_OPEN_READWRITE, 32632);
          m_searchGridCoverage = new ENCGIS::SearchGridCoverageState(m_con, std::string("coverage"));
        } catch(std::runtime_error& e) {
          err(DTR("Problem opening charts database: %s"), e.what());
          setEntityState(IMC::EntityState::ESTA_FAULT, Status::CODE_MISSING_DATA);
        }
        
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        if(m_searchGridCoverage != NULL)
          m_searchGridCoverage->deleteGrid();
        m_searchGridCoverage->createGrid(m_args.planningBounds[0], m_args.planningBounds[1], m_args.planningBounds[2], m_args.planningBounds[3], m_args.gridSize, ENCGIS::SearchGrid::gridtypes_t(m_args.gridType), true);
        inf("init");
      }

      //! Release resources.
      void
      onResourceRelease(void) {
        inf("Release");
        //if(m_searchGridCoverage != NULL)
        //  m_searchGridCoverage->deleteGrid();
        try {
          Memory::clear(m_con);
          Memory::clear(m_searchGridCoverage);
        }
          catch(std::runtime_error& e) {
          err(DTR("Could not clear charts database class: %s"), e.what());
        }
      }

      void consume(const IMC::EstimatedState* msg) {
        if(msg->getSource() != getSystemId()) {
          if(!monitoredVehicles.empty()) {
            if(std::find(monitoredVehicles.begin(), monitoredVehicles.end(), msg->getSource()) == monitoredVehicles.end()) {
              return;
            }
          }
          // Else update on all vehicles
        }
        pendingUpdates[msg->getSource()] = std::pair<double,double>(msg->lon, msg->lat);
      }

      void consume(const IMC::Rpm* msg) {
        rpm = std::abs(msg->value);
      }

      //! Main loop.
      void
      task(void)
      {
        //m_searchGridCoverage->decreaseAll(m_args.timestepDecrease);
        //m_searchGridCoverage->decreaseAll(m_args.timestepConstantDecrease, m_args.timestepFactorDecrease);
        consumeMessages();
        for(auto iter = pendingUpdates.begin();iter != pendingUpdates.end();iter++) {
          // Convert from WGS-84 to EPSG32632
          double northing, easting;
          m_con->transformSRID(Math::Angles::degrees(iter->second.first), Math::Angles::degrees(iter->second.second), 4326, easting, northing, 32632);
          //m_searchGridCoverage->update(easting, northing, m_args.initialSensorRange);
          m_searchGridCoverage->updateLogarithmic(easting, northing, rpm, (1/getFrequency())/7, 670); // Assuming 90 sec is needed to guarantee detection

          //inf("%f %f", easting, northing);
        }
        pendingUpdates.clear();
      }
    };
  }
}

DUNE_TASK
