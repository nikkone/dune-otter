#ifndef OMPL_FOR_DUNE_LIBRARY
#define OMPL_FOR_DUNE_LIBRARY

#define OMPL_BENCHMARK 1
// DUNE headers.
#include <DUNE/DUNE.hpp>

#include <ENCGIS/DBconnection.hpp>

// OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>



#if OMPL_BENCHMARK
#include <ompl/tools/benchmark/Benchmark.h> // Only used for benchmarking, remove before deployment
#endif

 #include <ompl/util/Time.h>
// CPP headers 
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace OMPLintegrationDUNE
{
  bool isStateValid(const ompl::base::State *state,  ENCGIS::isPointInLayerStatement* qry);
  //! @author Nikolai Lauvås
  void findPathExtended(og::SimpleSetup &ss);
  ompl::geometric::PathGeometric findPath(og::SimpleSetup &ss, double maxPlaningTime);
    ompl::geometric::SimpleSetup createSetup(double startLat, double startLon, double goalLat, double goalLon, ob::RealVectorBounds &bounds, ENCGIS::isPointInLayerStatement* pointCheck, ENCGIS::lineIntersectLayerStatement* lineCheck, unsigned searchDepth = 4);

  ompl::geometric::SimpleSetup createSetup2(double startLat, double startLon, double goalLat, double goalLon, ob::RealVectorBounds &bounds, ENCGIS::isPointInLayerStatement* pointCheck, ENCGIS::lineIntersectLayerStatement* lineCheck, ENCGIS::getClosestIntersectWithOffset* lineCheck2, unsigned searchDepth = 4);
  ompl::base::PlannerPtr kabitstar(const ompl::base::SpaceInformationPtr &si, std::string name);
  ompl::base::PlannerPtr kbitstar(const ompl::base::SpaceInformationPtr &si, std::string name);
  ompl::base::PlannerTerminationCondition exactSolnPlannerTerminationCondition(const bool *stop, DUNE::Tasks::Task *inTask, double maxPlaningTime);
  ompl::base::ReportIntermediateSolutionFn intermediate(DUNE::Tasks::Task *inTask, double minPlaningTime);
  #if OMPL_BENCHMARK
  void bmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount);
  void multiBmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount);

  #endif

}

#endif // OMPL_FOR_DUNE_LIBRARY