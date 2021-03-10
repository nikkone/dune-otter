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
// Author: Nikolai Lauvås                                                  *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <USER/DUNE.hpp>

namespace Supervisors
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Nikolai Lauvås
  namespace AntiGrounding
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! The path of the database
      std::string dbPath;
      //! Surroundings check frequency.
      float surroundingsCheckFreq;
      //! GPS entity label.
      std::string elabelGPS;
      //! GPS entity label.
      bool checkPlan;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Database connection
      ChartsDatabase::ChartsDBConnection* m_con;
      //! Surroundings Check Timer.
      Time::Counter<float> m_surr_timer;
      //! Current Lat and Lon of vehicle.
      double m_current_lat, m_current_lon;
      //! GPS entity eid.
      int m_gps_eid;


      //! Task arguments.
      Arguments m_args;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_con(NULL),
        m_current_lat(0.0),
        m_current_lon(0.0)
      {
        param("DB Path", m_args.dbPath)
        .defaultValue("")
        .description("Path of the db");

        param("Surroundings Check Frequency", m_args.surroundingsCheckFreq)
        .defaultValue("0.0")
        .description("How often to check for grounding or landing");

        param("GPS Entity Label", m_args.elabelGPS)
        .defaultValue("")
        .description("Source of GPSFix message");

        param("Check Activated Plan", m_args.checkPlan)
        .defaultValue("true")
        .description("Toggle to check safety of activated plans");

        bind<IMC::Abort>(this);
        bind<IMC::PlanSpecification>(this);
        bind<IMC::GpsFix>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.surroundingsCheckFreq))
          m_surr_timer.setTop(m_args.surroundingsCheckFreq);
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
        try
        {
          m_gps_eid = resolveEntity(m_args.elabelGPS);
        }
        catch (...)
        {
          m_gps_eid = 0;
        }
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        try{
          m_con = new ChartsDatabase::ChartsDBConnection(m_args.dbPath.c_str(), Database::Connection::CF_CREATE);
        } catch(std::runtime_error& e) {
          err(DTR("Problem opening charts database: %s"), e.what());
          // Set task state to failure
        }

      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        //! Set timer for periodic check of surroundings.
        m_surr_timer.setTop(m_args.surroundingsCheckFreq);
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

      void
      consume(const IMC::Abort* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (isActive())
          requestDeactivation();
      }

            void
      consume(const IMC::GpsFix* msg)
      {
        if(msg->getSource() != getSystemId() || msg->getSourceEntity() != m_gps_eid)
          return;
        m_current_lat=msg->lat;
        m_current_lon=msg->lon;

        /*if(m_timer.overflow())
        {
          //! Check vehicle surroundings.
          TwoDGrid::DepthVector a = m_nc->getWithinRadius(m_current_lat, m_current_lon, m_args.range);
          for (TwoDGrid::DepthVector::iterator itr = a.begin(); itr != a.end(); ++itr)
          {
            inf("%f %f %f", itr->Lat, itr->Lon, itr->Depth);
          }
          //! Is just a variable for debugging m_surr_num.
          m_nc->writeCSVfile(a, m_args.debug_path + "surroundings" + std::to_string(m_surr_num) + ".csv");
          m_surr_num++;

          m_timer.reset();
        }*/
      }

      void
      consume(const IMC::PlanSpecification* msg)
      {
        // TODO: Legg til currentpos
        if(m_args.checkPlan) {
          float minDepth = 10.0;
          float planMinDepth = m_con->checkPlanMinDepth(msg, m_current_lat, m_current_lon);
          if(planMinDepth < minDepth) {
            war("Aborted plan because of grounding, min depth: %f", planMinDepth);
              IMC::Abort abort;
              abort.setDestination(getSystemId());
              dispatch(abort);
          } else {
            if(m_con->checkPlanLanding(msg, m_current_lat, m_current_lon)) {
              war("Aborted plan because of landing, min depth: %f", planMinDepth);
              IMC::Abort abort;
              abort.setDestination(getSystemId());
              dispatch(abort);
            } else {
              inf("No grounding or landing detected, plan approved. Min depth: %f", planMinDepth);
            }
          }
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        //double minX=0,minY=0,maxX=0,maxY=0;
        /*std::string table = "deparetable";
        m_con->getExtent(table, minX,minY,maxX,maxY);
        inf("one");
        inf("%f %f %f %f",minX,minY,maxX,maxY);*/
        //ChartsDBConnection::RRTclass* alg = new ChartsDBConnection::RRTclass(m_con);
        //ChartsDBConnection::RRTStarClass* alg = new ChartsDBConnection::RRTStarClass(m_con);
        //alg->resetTree();
        //alg->setMaxStepLength(100.0);
        //alg->setExtent(10.369549,63.407093,10.426469,63.463678);
        //alg->setStartingPoint(63.44540, 10.38627);
        //alg->setGoalPoint(63.41434, 10.38903);
        
        //alg->run(1000);
        //alg->run(1000);
        //m_con->RRT(63.44540, 10.38627,63.41434, 10.38903);


        //m_con->RRT(63.46480, 10.48288,63.34213, 10.22443);
        // Straight Goal 63.4587, 10.0782, 63.43119, 10.02880
        // Byneset 63.4587, 10.0782,63.3442, 10.1182
        // Behind munkholmen 63.4587, 10.0782,63.45082, 10.38554

        //m_con->getRandomValidPointWithinExtent();
        //m_con->checkTransectMinDepth(63.448756, 10.418479, 63.44301, 10.42424);
        //m_con->checkTransectLanding(63.448756, 10.418479, 63.44301, 10.42424);
        /*std::ofstream file_("/home/nikolai/delme/rndpnt.csv");
        file_ << "Lon,Lat"  << "\r\n";
        std::pair<double, double>out;
        for(int i= 0;i<10000;i++) {
          out = m_con->getRandomValidPointWithinExtent();
          file_ <<std::setprecision(15) << out.first << ", " <<out.second << "\r\n"; */
        inf("Started");
        //}
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
