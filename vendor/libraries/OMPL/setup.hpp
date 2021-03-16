#ifndef OMPL_FOR_DUNE_LIBRARY
#define OMPL_FOR_DUNE_LIBRARY

#define OMPL_BENCHMARK 1
// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <ENCGIS/DBconnection.hpp>
#include <ENCGIS/DBTree.hpp>
#include <OMPL/OMPLfunctions.hpp>
#include <OMPL/OMPLMotionValidator.hpp>
#include <OMPL/OMPLMotionValidator2.hpp>
#include <OMPL/LatLonOptimizationObjective.hpp>

// OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
//#include "ompl/base/DiscreteMotionValidator.h"
#include <ompl/config.h>

#include <ompl/geometric/planners/informedtrees/ABITstar.h>

#if OMPL_BENCHMARK
#include <ompl/tools/benchmark/Benchmark.h> // Only used for benchmarking, remove before deployment
#endif

 #include <ompl/util/Time.h>
 #include <utility>
// #include <ctime>

// CPP headers
#include <iostream>
 
namespace ob = ompl::base;
namespace og = ompl::geometric;

#include <algorithm> 
#include <chrono> 
#include <iostream> 
#include<vector> 

namespace OMPLintegrationDUNE
{
  bool isStateValid(const ompl::base::State *state,  ENCGIS::isPointInLayerStatement* qry);
  //! @author Nikolai Lauvås
  void findPathExtended(og::SimpleSetup &ss);
  ompl::geometric::PathGeometric findPath(og::SimpleSetup &ss, double maxPlaningTime);
  ompl::geometric::SimpleSetup createSetup(double startLat, double startLon, double goalLat, double goalLon, ob::RealVectorBounds &bounds, ENCGIS::isPointInLayerStatement* pointCheck, ENCGIS::lineIntersectLayerStatement* lineCheck);
  ompl::base::PlannerPtr kabitstar(const ompl::base::SpaceInformationPtr &si, std::string name);
  ompl::base::PlannerTerminationCondition exactSolnPlannerTerminationCondition(const bool *stop, DUNE::Tasks::Task *inTask, double maxPlaningTime);
  ompl::base::ReportIntermediateSolutionFn intermediate(DUNE::Tasks::Task *inTask, double minPlaningTime);
  #if OMPL_BENCHMARK
  void bmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount);
  #endif

}

#endif // OMPL_FOR_DUNE_LIBRARY