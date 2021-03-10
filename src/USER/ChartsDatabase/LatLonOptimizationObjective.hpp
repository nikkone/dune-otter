#ifndef DUNE_CHARTSDATABASE_OMPL_LATLONOPTIMIZATIONOBJECTIVE_
#define DUNE_CHARTSDATABASE_OMPL_LATLONOPTIMIZATIONOBJECTIVE_

#include <ompl/base/OptimizationObjective.h>

namespace ob = ompl::base;

namespace DUNE
{
  namespace ChartsDatabase
  {
    class LatLonDist : public ob::OptimizationObjective
    {
      public:
        LatLonDist(const ob::SpaceInformationPtr &si);
        
        ompl::base::Cost stateCost(const ob::State *) const;
        
        ompl::base::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
        
        ompl::base::Cost motionCostHeuristic(const ob::State *s1, const ob::State *s2) const;

        ompl::base::InformedSamplerPtr allocInformedStateSampler(const ob::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const;

    };
  }
}

#endif // DUNE_CHARTSDATABASE_OMPL_LATLONOPTIMIZATIONOBJECTIVE_