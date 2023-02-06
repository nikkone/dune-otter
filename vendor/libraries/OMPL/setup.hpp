#ifndef OMPL_FOR_ENCGIS_LIBRARY
#define OMPL_FOR_ENCGIS_LIBRARY

// Build settings for disabling unneded/unsupported features
#define NEWOMPL 1
#define OMPL_BENCHMARK 0

// ENCGIS header
//#include <ENCGIS/DBconnection.hpp>
#include <ENCGIS/isPointInLayerStatement.hpp>
#include <ENCGIS/lineIntersectLayerStatement.hpp>


// OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/util/Time.h>
#if OMPL_BENCHMARK
#include <ompl/tools/benchmark/Benchmark.h> // Only used for benchmarking, remove before deployment
#endif

namespace ob = ompl::base;
namespace og = ompl::geometric;


//! @author Nikolai Lauvås
namespace OMPLintegrationENCGIS
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
  //! @param[in] startY The Y coordinate of the starting point of the search
  //! @param[in] startX The X coordinate of the starting point of the search
  //! @param[in] goalY The Y coordinate of the goal point of the search
  //! @param[in] goalX The X coordinate of the goal point of the search
  //! @param[in] bounds The planning bounds, where the samples are chosen from
  //! @param[in] pointCheck Point collision check/state validator
  //! @param[in] lineCheck Line collision check/motion validator
  //! @param[in] searchDepth Binary search depth used in motion validator. For most planners, use 1.
  //! @return An OMPL SimpleSetup object with start, goal, bounds and collision checks configured.
  ompl::geometric::SimpleSetup createSetup(double startY, double startX, double goalY, double goalX, ob::RealVectorBounds &bounds, ENCGIS::isPointInLayerStatement* pointCheck, ENCGIS::lineIntersectLayerStatement* lineCheck, unsigned searchDepth = 1);  
  

  ompl::geometric::SimpleSetup createSetup(double minY, double minX, double maxY, double maxX, ENCGIS::isPointInLayerStatement* pointCheck, ENCGIS::lineIntersectLayerStatement* lineCheck, unsigned searchDepth = 1);

  void setStartAndGoalStates(ompl::geometric::SimpleSetup &ss, double startY, double startX, double goalY, double goalX);

#if NEWOMPL
  //! Preconfigured kbitstar planner
  //! @param[in] si Space information pointer
  //! @param[in] name The name given to the planner
  //! @return Planner pointer to be used.
  ompl::base::PlannerPtr kbitstar(const ompl::base::SpaceInformationPtr &si, std::string name);
#endif
#if OMPL_BENCHMARK
  //! Runs a single planner benchmark and saves the results to file
  //! @param[in]ss Simple setup object configured with boundaries, initial/goal locations and motion/state validators
  //! @param[in]benchmark_name Name of benchmark
  //! @param[in]benchmark_maxTime Max time per run
  //! @param[in]benchmark_maxMem Max memory usage. It does not seem like memory is released before benchmark is finnished, so set as high as possible
  //! @param[in]benchmark_runCount How many runs is performed per planner
  void bmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount);
  //! Runs a mulit-planner benchmark and saves the results to file
  //! @param[in]ss Simple setup object configured with boundaries, initial/goal locations and motion/state validators
  //! @param[in]benchmark_name Name of benchmark
  //! @param[in]benchmark_maxTime Max time per run
  //! @param[in]benchmark_maxMem Max memory usage. It does not seem like memory is released before benchmark is finnished, so set as high as possible
  //! @param[in]benchmark_runCount How many runs is performed per planner
  void multiBmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount);
#endif

}

#endif // OMPL_FOR_ENCGIS_LIBRARY
