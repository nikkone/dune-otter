//***************************************************************************
// Copyright 2020-2021 Norwegian University of Science and Technology       *
// Department of Engineering Technology                                     *
//***************************************************************************
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

#include "OMPLfunctions.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <DUNE/Coordinates/UTM.hpp>

  //! @author Nikolai Lauvås
  namespace OMPLforDUNE
  {
    void printPath(og::PathGeometric states) {
        for(unsigned i=0;i<states.getStateCount();i++) {
        const auto state= static_cast<const ompl::base::RealVectorStateSpace::StateType *>(states.getState(i));      
        std::cout << "State " << i << ": " << state->values[0] << " " << state->values[1] << std::endl;
        }
    }

    void pathToTree(og::PathGeometric states, std::string treeName, ENCGIS::DBTree* tree) {
        //try{
            tree->resetTree(treeName);
        //} catch(...) {
        //    err("treeName cant be reset");
        //    return;
        //}
        for(unsigned i=0;i<states.getStateCount();i++) {
            const auto state= static_cast<const ompl::base::RealVectorStateSpace::StateType *>(states.getState(i));
            if(i!=0)
                tree->insertNode(treeName,i,state->values[1], state->values[0]);
            else
            {
                tree->insertNode(treeName,1,state->values[1], state->values[0]);
            }
        }
    }

      ompl::base::PlannerTerminationCondition exactSolnPlannerTerminationCondition(const bool *stop, DUNE::Tasks::Task *inTask, double maxPlaningTime)
  {
    double duration = maxPlaningTime; // Maxtime
    const ompl::time::point endTime(ompl::time::now() + ompl::time::seconds(duration));
    return ompl::base::PlannerTerminationCondition([stop, inTask, endTime]
      {
        if(*stop) {
          return true;
        }

        if(ompl::time::now() > endTime){
          inTask->inf("maxtime");
          return true;
        }

        return inTask->isStopping();
      }
    );
  }


  
  ompl::base::ReportIntermediateSolutionFn intermediate(DUNE::Tasks::Task *inTask, double minPlaningTime) {
    double duration = minPlaningTime; // Mintime
    const ompl::time::point endTime(ompl::time::now() + ompl::time::seconds(duration));
    return ompl::base::ReportIntermediateSolutionFn([inTask, endTime](const ob::Planner *planner, const std::vector< const ob::State * > &states, const ob::Cost cost) 
    { 
      if(ompl::time::now() > endTime){
        inTask->inf("mintime");
      }
      // Create ompl::geometric::PathGeometric from "states" vector
      auto path = ompl::geometric::PathGeometric(planner->getSpaceInformation());
      for (auto ptr = states.begin(); ptr < states.end(); ptr++) {
        //auto state = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(*ptr);
        //std::cout << state->values[1] <<","<< state->values[0] << std::endl;
        path.append(*ptr);
      }

          //ENCGIS::DBTree* tree = new ENCGIS::DBTree(m_con);
          //MotionPlanners::OMPL::pathToTree(path, "tree", tree);
          //Memory::clear(tree);

      //intermediate(planner,states, cost); 
      inTask->inf("intermediate solution with %f Cost", cost.value());
    //std::cout << "intermediate solution with "<< cost.value() << "Cost"<< std::endl;
    //m_intermediate = true;
    // Check if mintime reached, else return false

    });
  }
//////////////////////////////////////////// Plan Generation
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



    DUNE::IMC::PlanDB createPlanDBEntry(og::PathGeometric paths, std::string plan_id, fp32_t speed) {

    DUNE::IMC::MessageList<DUNE::IMC::Maneuver> maneuvers; //Define list of meneuvers
        // Make maneuvers
        for(unsigned i=0;i<paths.getStateCount();i++) {
            const auto state= static_cast<const ompl::base::RealVectorStateSpace::StateType *>(paths.getState(i));
            DUNE::IMC::Goto* go_near = new DUNE::IMC::Goto();
            go_near->lat = DUNE::Math::Angles::radians(state->values[0]);
            go_near->lon = DUNE::Math::Angles::radians(state->values[1]);
            go_near->speed_units = DUNE::IMC::SUNITS_METERS_PS;
            go_near->speed = speed;//m_args.speed_rpms;
            maneuvers.push_back(*go_near);

            delete go_near;
            //std::cout << "State " << i << ": " << state->values[0] << " " << state->values[1] << std::endl;
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

    DUNE::IMC::PlanDB createPlanDBEntryUTM(og::PathGeometric paths, std::string plan_id, fp32_t speed, int zone) {

      DUNE::IMC::MessageList<DUNE::IMC::Maneuver> maneuvers; //Define list of meneuvers
        // Make maneuvers
        for(unsigned i=0;i<paths.getStateCount();i++) {
            const auto state= static_cast<const ompl::base::RealVectorStateSpace::StateType *>(paths.getState(i));
            DUNE::IMC::Goto* go_near = new DUNE::IMC::Goto();
            DUNE::Coordinates::UTM::toWGS84(state->values[0], state->values[1], zone, true, &(go_near->lat), &(go_near->lon));
            //go_near->lat = DUNE::Math::Angles::radians(state->values[1]);
            //go_near->lon = DUNE::Math::Angles::radians(state->values[0]);
            go_near->speed_units = DUNE::IMC::SUNITS_METERS_PS;
            go_near->speed = speed;//m_args.speed_rpms;
            maneuvers.push_back(*go_near);

            delete go_near;
            //std::cout << "State " << i << ": " << state->values[0] << " " << state->values[1] << std::endl;
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

    bool isStateValid(const ompl::base::State *state,  ENCGIS::DBconnection *dbCon)
    {
        if (state != nullptr)
        {
            const auto *rstate = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(state);
            return dbCon->isPointInLayer(rstate->values[1], rstate->values[0]);
        }
        else
            std::cout << "nullptr" << std::endl;
        return false;
        
    }

    void OutputHandlerDUNEConsole::log(const std::string &text, ompl::msg::LogLevel level, const char *filename, int line) {
      switch(level) {
        case (ompl::msg::LOG_DEV2): 
          task->spew("OMPL::%s", text.c_str());
          break;
        case (ompl::msg::LOG_DEV1):
          task->trace("OMPL::%s", text.c_str());
          break;
        case (ompl::msg::LOG_DEBUG):
          task->debug("OMPL::%s", text.c_str());
          break;
        case (ompl::msg::LOG_INFO):
          task->inf("OMPL::%s", text.c_str());
          break;
        case (ompl::msg::LOG_WARN):
          task->war("OMPL::%s", text.c_str());
          break;
        case (ompl::msg::LOG_ERROR):
          task->err("OMPL::%s", text.c_str());
          break;
        case (ompl::msg::LOG_NONE):
          task->spew("OMPL::%s", text.c_str());
          break;
        default:
          task->inf("OMPL::%s", text.c_str());
      }
    }
  }

