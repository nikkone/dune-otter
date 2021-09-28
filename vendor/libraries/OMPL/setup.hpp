#ifndef OMPL_FOR_DUNE_LIBRARY
#define OMPL_FOR_DUNE_LIBRARY

#define NEWOMPL 1
#define OMPL_BENCHMARK 0
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


//! @author Nikolai Lauvås
namespace OMPLintegrationDUNE
{
  //! Enumeration for created OMPL/planner configurations
  enum configurations_t {C_DEFAULT, C_KBIT, C_KABIT, C_FMT};

  //! Function that adapts the ENCGIS::isPointInLayerStatement interface to the OMPL state validator interface
  //! @param[in] state The geometric state to be checked for validity
  //! @param[in] qry Class containing persistent database statement that is run.
  bool isStateValid(const ompl::base::State *state,  ENCGIS::isPointInLayerStatement* qry);

  //! Selects planner config and runs the planning algorithm
  //! @param[in] ss Simple setup object configured with boundaries, initial/goal locations and motion/state validators
  //! @param[in] maxPlaningTime Force end planning when duration is longer than this.
  //! @param[in] config The chosen OMPL/planner configuration
  //! @return Geometric path object with feasible path, or if timed out, an empty path.
  ompl::geometric::PathGeometric findPath(og::SimpleSetup &ss, double maxPlaningTime, configurations_t config = C_DEFAULT);

  //! Creates a simplesetup object using the ChartsDB polygons for state and motion validation.
  //! @param[in] startY
  //! @param[in] startX
  //! @param[in] goalY
  //! @param[in] goalX
  //! @param[in] bounds
  //! @param[in] pointCheck
  //! @param[in] lineCheck
  //! @param[in] searchDepth
  //! @return 
  ompl::geometric::SimpleSetup createSetup(double startY, double startX, double goalY, double goalX, ob::RealVectorBounds &bounds, ENCGIS::isPointInLayerStatement* pointCheck, ENCGIS::lineIntersectLayerStatement* lineCheck, unsigned searchDepth = 1);  
  
  ompl::base::PlannerPtr kabitstar(const ompl::base::SpaceInformationPtr &si, std::string name);
  ompl::base::PlannerPtr kbitstar(const ompl::base::SpaceInformationPtr &si, std::string name);

  //! Termination condition to be used with ompl. This one stops the planner when the task gets the stop signal, if the stop input is true or if maxTime is reached.
  ompl::base::PlannerTerminationCondition exactSolnPlannerTerminationCondition(const bool *stop, DUNE::Tasks::Task *inTask, double maxPlaningTime);
  ompl::base::ReportIntermediateSolutionFn intermediate(DUNE::Tasks::Task *inTask, double minPlaningTime);
  #if OMPL_BENCHMARK
  void bmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount);
  void multiBmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount);

  #endif

}

#endif // OMPL_FOR_DUNE_LIBRARY