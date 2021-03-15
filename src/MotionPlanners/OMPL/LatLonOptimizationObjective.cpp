#include "LatLonOptimizationObjective.hpp"
#include <DUNE/Math/Angles.hpp>
#include <DUNE/Coordinates/WGS84.hpp>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
MotionPlanners::OMPL::LatLonDist::LatLonDist(const ob::SpaceInformationPtr &si)
   : ompl::base::OptimizationObjective(si)
 {
     description_ = "Lengt from WGS84 lat/lon pairs";
  
     // Setup a default cost-to-go heuristics:
     setCostToGoHeuristic(ob::goalRegionCostToGo);
 }
  
ompl::base::Cost MotionPlanners::OMPL::LatLonDist::stateCost(const ob::State *) const
 {
     return identityCost();
 }
  
ompl::base::Cost MotionPlanners::OMPL::LatLonDist::motionCost(const ob::State *s1, const ob::State *s2) const
 {
   //std::cout << "Direct" << std::endl;
    const double *sd1 = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(s1)->values;
    const double *sd2 = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(s2)->values;
    return ob::Cost(DUNE::Coordinates::WGS84::distance(DUNE::Math::Angles::radians(sd1[1]),DUNE::Math::Angles::radians(sd1[0]),0.0,DUNE::Math::Angles::radians(sd2[1]),DUNE::Math::Angles::radians(sd2[0]),0.0));
 }
  
ompl::base::Cost MotionPlanners::OMPL::LatLonDist::motionCostHeuristic(const ob::State *s1, const ob::State *s2) const
 {
   //std::cout << "Heuristic" << std::endl;
     return ob::Cost(si_->distance(s1, s2));
 }

 ompl::base::InformedSamplerPtr MotionPlanners::OMPL::LatLonDist::allocInformedStateSampler(const ob::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const
 {
 // Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct version is
 // available, if not a rejection-based technique can be used
     return std::make_shared<ob::PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
 }
