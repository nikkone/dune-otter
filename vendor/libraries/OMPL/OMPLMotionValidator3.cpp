#include "OMPLMotionValidator3.hpp"
#include <ompl/util/Exception.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;


void MotionPlanners::OMPL::ChartsDBMotionValidator3::defaultSettings()
{
    stateSpace_ = si_->getStateSpace().get();
    if (stateSpace_ == nullptr)
        throw ompl::Exception("No state space for motion validator");
}

bool MotionPlanners::OMPL::ChartsDBMotionValidator3::checkMotion(const ob::State *s1, const ob::State *s2,
                                                      std::pair<ob::State *, double> &lastValid) const
{
    
    // If lastValid.first is nullPtr, then the exact state is dontcare? OMPL discreteMotionValidator also does this.
    // assume motion starts in a valid configuration so s1 is valid 
    double lastValidX;
    double lastValidY;
    double lastValidTime = transectSafety(s1,s2, lastValidX, lastValidY);
    bool result = false;
    if(lastValidTime < 1.0) {
        if(lastValidTime != 0.0) {
            lastValid.second = lastValidTime;
            ob::State *test = si_->allocState();
            stateSpace_->interpolate(s1, s2, lastValidTime, test);
            stateSpace_->copyState(lastValid.first, s1);
            si_->freeState(test);
        }

    } else {
        result=true;
    }


    if (result)
        valid_++;
    else
        invalid_++;
    return result;
}


bool MotionPlanners::OMPL::ChartsDBMotionValidator3::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
   const auto *state1 = static_cast<const ob::RealVectorStateSpace::StateType *>(s1);
    const auto *state2 = static_cast<const ob::RealVectorStateSpace::StateType *>(s2);
    if(dbcon->run(state1->values[1], state1->values[0], state2->values[1], state2->values[0]) == 0) {
        valid_++;
        return true;
    } else {
        invalid_++;
        return false;
    }
}

double MotionPlanners::OMPL::ChartsDBMotionValidator3::transectSafety(const ob::State *s1, const ob::State *s2, double &bestOptionX, double &bestOptionY) const {

    const auto *state1 = static_cast<const ob::RealVectorStateSpace::StateType *>(s1);
    const auto *state2 = static_cast<const ob::RealVectorStateSpace::StateType *>(s2);
    return dbcon2->run(state1->values[1], state1->values[0], state2->values[1], state2->values[0], bestOptionX, bestOptionY);
}